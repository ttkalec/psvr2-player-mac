// Offscreen GPU regression: the corrected UI must match three independent
// monochromatic renders through the corresponding optics. A constant LUT
// isolates channel routing, including alpha coverage and the virtual cursor.
import Foundation
import Metal
import simd

struct TestUniforms {
    var rot = matrix_identity_float4x4
    var panelInv = matrix_identity_float4x4
    var calibL = SIMD4<Float>(-0.09919293, 0, 1, 0)
    var calibR = SIMD4<Float>(0.09919293, 0, 1, 0)
    var p0 = SIMD4<Float>(0, 0, .pi, 1)
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
let source = try String(contentsOfFile: CommandLine.arguments[1], encoding: .utf8)
let shader = source.components(separatedBy: "let shaderSource = #\"\"\"\n")[1]
    .components(separatedBy: "\n\"\"\"#")[0]
let library = try device.makeLibrary(source: shader, options: nil)
let pipelineDescriptor = MTLRenderPipelineDescriptor()
pipelineDescriptor.vertexFunction = library.makeFunction(name: "vs_main")
pipelineDescriptor.fragmentFunction = library.makeFunction(name: "fs_main")
pipelineDescriptor.colorAttachments[0].pixelFormat = .rgba32Float
let pipeline = try device.makeRenderPipelineState(descriptor: pipelineDescriptor)
let width = 640, height = 320, uiWidth = 128, uiHeight = 64

func texture(_ w: Int, _ h: Int, target: Bool = false) -> MTLTexture {
    let descriptor = MTLTextureDescriptor.texture2DDescriptor(
        pixelFormat: .rgba32Float, width: w, height: h, mipmapped: false)
    descriptor.storageMode = .shared
    descriptor.usage = target ? [.renderTarget] : [.shaderRead]
    return device.makeTexture(descriptor: descriptor)!
}
let ui = texture(uiWidth, uiHeight)
let placeholder = texture(1, 1)
var black = SIMD4<Float>(repeating: 0)
placeholder.replace(region: MTLRegionMake2D(0, 0, 1, 1), mipmapLevel: 0,
                    withBytes: &black, bytesPerRow: 16)
let target = texture(width, height, target: true)
let scales: [Float] = [0.82, 0.86, 0.90]

func render(_ uniforms: TestUniforms, _ optics: [Float]) -> [Float] {
    // packed_float3 in the shader, so the LUT has exactly three floats per row.
    let lut = Array(repeating: optics, count: 1024).flatMap { $0 }
    let buffer = lut.withUnsafeBytes {
        device.makeBuffer(bytes: $0.baseAddress!, length: $0.count, options: .storageModeShared)!
    }
    let pass = MTLRenderPassDescriptor()
    pass.colorAttachments[0].texture = target
    pass.colorAttachments[0].loadAction = .dontCare
    pass.colorAttachments[0].storeAction = .store
    let command = queue.makeCommandBuffer()!
    let encoder = command.makeRenderCommandEncoder(descriptor: pass)!
    encoder.setRenderPipelineState(pipeline)
    var uniforms = uniforms
    encoder.setFragmentBytes(&uniforms, length: MemoryLayout<TestUniforms>.stride, index: 0)
    encoder.setFragmentBuffer(buffer, offset: 0, index: 1)
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

for fixture in ["opaque strokes", "translucent strokes", "cursor", "hidden panel"] {
    var pixels = [SIMD4<Float>](repeating: .zero, count: uiWidth * uiHeight)
    for y in 2..<(uiHeight - 2) {
        for x in 2..<(uiWidth - 2) {
            let stroke = x % 13 < 3 || y % 17 < 2
            let alpha: Float = fixture == "cursor" ? 0 : (stroke ? 1 : 0.2)
            let coverage = fixture == "translucent strokes" ? alpha * 0.45 : alpha
            pixels[y * uiWidth + x] = SIMD4(coverage * 0.9, coverage * 0.7, coverage * 0.5, coverage)
        }
    }
    pixels.withUnsafeBytes {
        ui.replace(region: MTLRegionMake2D(0, 0, uiWidth, uiHeight), mipmapLevel: 0,
                   withBytes: $0.baseAddress!, bytesPerRow: uiWidth * 16)
    }
    var uniforms = TestUniforms()
    // Exercise panel rotation, eye calibration rotation, and shifted placement.
    uniforms.rot = float4x4(simd_quatf(angle: 0.12, axis: SIMD3(0, 1, 0)))
    uniforms.panelInv = float4x4(simd_quatf(angle: -0.08, axis: SIMD3(1, 0, 0)))
    uniforms.calibL.z = cos(0.04); uniforms.calibL.w = sin(0.04)
    uniforms.calibR.z = cos(-0.05); uniforms.calibR.w = sin(-0.05)
    uniforms.p3.x = 0.18
    if fixture == "cursor" { uniforms.p2.z = 0.8; uniforms.p2.w = 0.4 }
    if fixture == "hidden panel" { uniforms.p2.y = 0 }
    let actual = render(uniforms, scales)
    var errors = 0, shiftedPixels = 0
    var uncorrected = uniforms
    uncorrected.p2.x = 0
    let disabled = render(uncorrected, scales)
    for channel in 0..<3 {
        var reference = uncorrected
        // The green-only path has a vertical calibration offset. Cancel it
        // when using that path to render the red or blue reference optics.
        if channel != 1 { reference.calibL.y += 0.0002302693; reference.calibR.y += 0.0002302693 }
        let expected = render(reference, Array(repeating: scales[channel], count: 3))
        for y in 0..<height {
            for x in 0..<width {
                let eye = x < width / 2 ? 0 : 1
                let eyeU = (Float(x % (width / 2)) + 0.5) / Float(width / 2)
                let localX = (eye == 0 ? uniforms.calibL.x : uniforms.calibR.x)
                    + (eyeU * 2 - 1) * 0.9803922 * 0.9394987
                let localY = ((Float(y) + 0.5) / Float(height) * 2 - 1) * 0.9394987
                // Stay within the synthetic constant LUT, excluding extrapolation.
                if localX * localX + localY * localY > 0.9 { continue }
                let i = (y * width + x) * 4 + channel
                if abs(actual[i] - expected[i]) > 0.001 { errors += 1 }
                if abs(actual[i] - disabled[i]) > 0.01 { shiftedPixels += 1 }
            }
        }
    }
    require(errors == 0, "\(fixture): \(errors) pixels differ from independent color renders")
    require(fixture == "hidden panel" ? shiftedPixels == 0 : shiftedPixels > 100,
            "\(fixture): correction must affect visible UI and leave a hidden UI alone")
    print("PASS: \(fixture), both eyes, per-channel color and alpha")
}
