// GPU regression for the headset shader's speed-ups: the polynomial atan2/asin
// must match double precision, and the green-ray HUD early-out must leave the
// image identical to compositing every ray, with the real lens table at the
// extreme head poses the panel allows before re-anchoring.
import Foundation
import Metal
import simd

struct TestUniforms {
    var rot = matrix_identity_float4x4
    var panelInv = matrix_identity_float4x4
    var calibL = SIMD4<Float>(-0.10389773, 0.0041853427, 0.999997, -0.0024385462)
    var calibR = SIMD4<Float>(0.099605516, 0.0022621974, 0.9999959, 0.0028766221)
    var p0 = SIMD4<Float>(1, 1, .pi, 1)
    var p1 = SIMD4<Float>(repeating: 0)
    var p2 = SIMD4<Float>(1, 1, -10, -10)
    var p3 = SIMD4<Float>(0, 0, 0.5, 0.25)
    var p4 = SIMD4<Float>(repeating: 0)
    var p5 = SIMD4<Float>(repeating: 0)
    var p6 = SIMD4<Float>(repeating: 0)
}

func require(_ condition: Bool, _ message: String) {
    if !condition {
        fputs("FAIL: \(message)\n", stderr)
        exit(1)
    }
}

guard let device = MTLCreateSystemDefaultDevice(), let queue = device.makeCommandQueue() else {
    fatalError("A Metal device is required for the shader regression test")
}
let playerDir = URL(fileURLWithPath: CommandLine.arguments[1]).deletingLastPathComponent().path
let source = try String(contentsOfFile: CommandLine.arguments[1], encoding: .utf8)
let shader = source.components(separatedBy: "let shaderSource = #\"\"\"\n")[1]
    .components(separatedBy: "\n\"\"\"#")[0]

// MARK: Polynomial projection math against double precision

let probe = shader + """

kernel void trig_probe(device const float2 *in [[buffer(0)]], device float2 *out [[buffer(1)]],
                       uint i [[thread_position_in_grid]]) {
    out[i] = float2(atan2_poly(in[i].x, in[i].y), asin_poly(clamp(in[i].x, -1.0, 1.0)));
}
"""
let probeLibrary = try device.makeLibrary(source: probe, options: nil)
let probePipeline = try device.makeComputePipelineState(function: probeLibrary.makeFunction(name: "trig_probe")!)
// Every quadrant and octant boundary, plus exact axes and asin's endpoints.
var inputs: [SIMD2<Float>] = [
    SIMD2(0, 1), SIMD2(0, -1), SIMD2(1, 0), SIMD2(-1, 0), SIMD2(1, 1), SIMD2(-1, -1),
    SIMD2(1, -1), SIMD2(-1, 1), SIMD2(0, 0),
]
for i in 0..<4096 {
    let angle = Double(i) / 4096 * 2 * .pi
    for radius in [0.001, 0.7, 1.0, 40.0] {
        inputs.append(SIMD2(Float(radius * sin(angle)), Float(radius * cos(angle))))
    }
}
for i in 0...2000 { inputs.append(SIMD2(Float(i) / 1000 - 1, 1)) }
let inputBuffer = device.makeBuffer(bytes: inputs, length: inputs.count * 8, options: .storageModeShared)!
let outputBuffer = device.makeBuffer(length: inputs.count * 8, options: .storageModeShared)!
let probeCommand = queue.makeCommandBuffer()!
let probeEncoder = probeCommand.makeComputeCommandEncoder()!
probeEncoder.setComputePipelineState(probePipeline)
probeEncoder.setBuffer(inputBuffer, offset: 0, index: 0)
probeEncoder.setBuffer(outputBuffer, offset: 0, index: 1)
probeEncoder.dispatchThreads(MTLSize(width: inputs.count, height: 1, depth: 1),
                             threadsPerThreadgroup: MTLSize(width: 64, height: 1, depth: 1))
probeEncoder.endEncoding()
probeCommand.commit()
probeCommand.waitUntilCompleted()
let results = outputBuffer.contents().bindMemory(to: SIMD2<Float>.self, capacity: inputs.count)
var atanError = 0.0, asinError = 0.0
for (i, input) in inputs.enumerated() {
    let y = Double(input.x), x = Double(input.y)
    // ±π are the same direction (the 360° seam).
    let difference = abs(Double(results[i].x) - atan2(y, x))
    if !(x == 0 && y == 0) { atanError = max(atanError, min(difference, 2 * .pi - difference)) }
    asinError = max(asinError, abs(Double(results[i].y) - asin(max(-1, min(1, y)))))
}
// 1e-5 rad is 0.012 px of 8K 360° video (8.2e-4 rad per pixel).
require(atanError < 1e-5, "atan2_poly error \(atanError) rad")
require(asinError < 1e-5, "asin_poly error \(asinError) rad")
print(String(format: "PASS: projection math within %.1e rad (atan2) and %.1e rad (asin)", atanError, asinError))

// MARK: HUD early-out against compositing every ray

func pipeline(_ source: String) throws -> MTLRenderPipelineState {
    let library = try device.makeLibrary(source: source, options: nil)
    let descriptor = MTLRenderPipelineDescriptor()
    descriptor.vertexFunction = library.makeFunction(name: "vs_main")
    descriptor.fragmentFunction = library.makeFunction(name: "fs_main")
    descriptor.colorAttachments[0].pixelFormat = .rgba32Float
    return try device.makeRenderPipelineState(descriptor: descriptor)
}
func replacing(_ text: String, _ old: String, _ new: String) -> String {
    require(text.components(separatedBy: old).count == 2,
            "the HUD early-out changed; update the reference substitution in this test")
    return text.replacingOccurrences(of: old, with: new)
}
// Reference: no green pre-test, every ray computes its own panel position.
var everyRay = replacing(shader, """
        && panel_uv(local_x * scale[1], (local_y - 0.0002302693) * scale[1], k3, k4, disp, uni, greenUV)
        && greenUV.x >= -0.25 && greenUV.x <= 1.25 && greenUV.y >= -0.45 && greenUV.y <= 1.45;
""", ";")
everyRay = replacing(everyRay, """
        float2 puv = greenUV;
        if (ch != 1 && !panel_uv(local_x * scale[ch], local_y * scale[ch], k3, k4, disp, uni, puv)) {
""", """
        float2 puv;
        bool hit = ch == 1
            ? panel_uv(local_x * scale[1], (local_y - 0.0002302693) * scale[1], k3, k4, disp, uni, puv)
            : panel_uv(local_x * scale[ch], local_y * scale[ch], k3, k4, disp, uni, puv);
        if (!hit) {
""")
let fast = try pipeline(shader)
let reference = try pipeline(everyRay)

// The production lens table, as the headset reports it.
let table = try String(contentsOfFile: playerDir + "/lut.c", encoding: .utf8)
let tableBody = table.components(separatedBy: "lookup[] = {")[1].components(separatedBy: "};")[0]
let lut = tableBody.split(whereSeparator: { "{}, \n\t".contains($0) }).compactMap { Float($0) }
require(lut.count == 3072, "lens table must have 1024 RGB rows")
let lutBuffer = lut.withUnsafeBytes {
    device.makeBuffer(bytes: $0.baseAddress!, length: $0.count, options: .storageModeShared)!
}

let width = 1000, height = 510
func texture(_ w: Int, _ h: Int, target: Bool = false) -> MTLTexture {
    let descriptor = MTLTextureDescriptor.texture2DDescriptor(
        pixelFormat: .rgba32Float, width: w, height: h, mipmapped: false)
    descriptor.storageMode = .shared
    descriptor.usage = target ? [.renderTarget] : [.shaderRead]
    return device.makeTexture(descriptor: descriptor)!
}
// Opaque to the edge: clamp_to_edge then fills the whole margin, so any
// clipped ray shows up as a different pixel.
let ui = texture(64, 32)
var uiPixels = [SIMD4<Float>](repeating: SIMD4(0.9, 0.6, 0.3, 1), count: 64 * 32)
uiPixels.withUnsafeBytes {
    ui.replace(region: MTLRegionMake2D(0, 0, 64, 32), mipmapLevel: 0,
               withBytes: $0.baseAddress!, bytesPerRow: 64 * 16)
}
let placeholder = texture(1, 1)
let target = texture(width, height, target: true)

func render(_ state: MTLRenderPipelineState, _ uniforms: TestUniforms) -> [Float] {
    let pass = MTLRenderPassDescriptor()
    pass.colorAttachments[0].texture = target
    pass.colorAttachments[0].loadAction = .dontCare
    pass.colorAttachments[0].storeAction = .store
    let command = queue.makeCommandBuffer()!
    let encoder = command.makeRenderCommandEncoder(descriptor: pass)!
    encoder.setRenderPipelineState(state)
    var uniforms = uniforms
    encoder.setFragmentBytes(&uniforms, length: MemoryLayout<TestUniforms>.stride, index: 0)
    encoder.setFragmentBuffer(lutBuffer, offset: 0, index: 1)
    for index in 0..<5 { encoder.setFragmentTexture(index == 1 ? ui : placeholder, index: index) }
    encoder.drawPrimitives(type: .triangle, vertexStart: 0, vertexCount: 3)
    encoder.endEncoding()
    command.commit()
    command.waitUntilCompleted()
    require(command.status == .completed, "GPU render failed: \(String(describing: command.error))")
    var pixels = [Float](repeating: 0, count: width * height * 4)
    pixels.withUnsafeMutableBytes {
        target.getBytes($0.baseAddress!, bytesPerRow: width * 16,
                        from: MTLRegionMake2D(0, 0, width, height), mipmapLevel: 0)
    }
    return pixels
}

var poses = 0
for yaw in [-69.0, -45.0, 0.0, 45.0, 69.0] {
    for pitch in [-69.0, -45.0, 0.0, 45.0, 69.0] {
        // The renderer moves the panel to the gaze beyond ~70° (dot < 0.35).
        guard cos(yaw * .pi / 180) * cos(pitch * .pi / 180) >= 0.35 else { continue }
        for roll in [-90.0, -45.0, 0.0, 45.0, 90.0] {
            let relative = simd_quatf(angle: Float(yaw * .pi / 180), axis: SIMD3(0, 1, 0))
                * simd_quatf(angle: Float(pitch * .pi / 180), axis: SIMD3(1, 0, 0))
                * simd_quatf(angle: Float(roll * .pi / 180), axis: SIMD3(0, 0, 1))
            var uniforms = TestUniforms()
            uniforms.rot = float4x4(relative)
            for cursor in [false, true] {
                uniforms.p2.z = cursor ? 0.02 : -10
                uniforms.p2.w = cursor ? 0.97 : -10
                let actual = render(fast, uniforms)
                let expected = render(reference, uniforms)
                var differing = 0
                for i in 0..<actual.count where abs(actual[i] - expected[i]) > 1e-6 { differing += 1 }
                require(differing == 0, String(format:
                    "HUD early-out changed %d values at yaw %.0f° pitch %.0f° roll %.0f°",
                    differing, yaw, pitch, roll))
                poses += 1
            }
        }
    }
}
print("PASS: HUD early-out matches every-ray compositing at \(poses) extreme poses")
