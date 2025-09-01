#!/usr/bin/env python3
"""
Compile TFLite model for ESP32 using Apache TVM/MicroTVM
This optimizes memory usage significantly compared to TFLite Micro
"""

import os
import numpy as np
import tvm
from tvm import relay, runtime
from tvm.contrib import graph_executor
import tvm.micro
from tvm.micro import export_model_library_format

def compile_model_for_esp32(tflite_path, output_dir="tvm_model"):
    """
    Compile TFLite model to optimized C code using TVM.
    
    Args:
        tflite_path: Path to quantized .tflite model
        output_dir: Directory for generated C code
    """
    
    # Load the TFLite model
    with open(tflite_path, "rb") as f:
        tflite_model = f.read()
    
    # Import TFLite model to Relay (TVM's IR)
    import tflite
    tflite_model_buf = tflite.Model.GetRootAsModel(tflite_model, 0)
    
    # Parse TFLite model
    try:
        # Import the model
        mod, params = relay.frontend.from_tflite(
            tflite_model_buf,
            shape_dict={"input": (1, 3, 96, 96)},  # Channels-first format
            dtype_dict={"input": "int8"}
        )
    except Exception as e:
        print(f"Note: May need to install tflite package: pip install tflite")
        # Alternative approach using TVM's built-in parser
        import tvm.relay.frontend.tflite as tflite_frontend
        mod, params = tflite_frontend.from_tflite(
            tflite_model,
            shape_dict={"input": (1, 3, 96, 96)},
            dtype_dict={"input": "int8"}
        )
    
    print("Model imported to TVM Relay successfully")
    
    # Target ESP32 (Xtensa LX6)
    # Use C backend with microcontroller optimizations
    target = tvm.target.Target(
        "c "
        "-keys=cpu "
        "-mcpu=esp32 "
        "-runtime=c "
        "-executor=aot "  # Ahead-of-time compilation
        "-interface-api=c "
        "-unpacked-api=1"
    )
    
    # Configure for minimal memory usage
    with tvm.transform.PassContext(
        opt_level=3,  # Maximum optimization
        config={
            "tir.disable_vectorize": True,  # ESP32 doesn't have SIMD
            "tir.usmp.enable": True,  # Unified Static Memory Planning
            "tir.usmp.algorithm": "greedy_by_conflicts",  # Memory reuse algorithm
        }
    ):
        # Compile the model
        print("Compiling model with memory optimizations...")
        executor = relay.build_module.create_executor(
            "aot",  # Ahead-of-time compilation
            mod,
            target=target,
            params=params
        )
    
    # Generate C source code
    print(f"Generating C code in {output_dir}/")
    os.makedirs(output_dir, exist_ok=True)
    
    # Export Model Library Format (MLF)
    mlf_path = f"{output_dir}/model.tar"
    export_model_library_format(executor.module, mlf_path)
    
    # Extract and create standalone C files
    import tarfile
    with tarfile.open(mlf_path, "r") as tar:
        tar.extractall(output_dir)
    
    # Create main inference file
    create_inference_wrapper(output_dir)
    
    print(f"\nCompilation complete!")
    print(f"Generated files in: {output_dir}/")
    print(f"Memory usage will be significantly lower than TFLite Micro")
    
    return executor

def create_inference_wrapper(output_dir):
    """Create a simple C++ wrapper for the TVM model."""
    
    wrapper_code = '''// TVM-optimized model for ESP32
// This uses much less memory than TFLite Micro

#include <stdint.h>
#include <esp_log.h>
#include "tvm/runtime/c_runtime_api.h"
#include "tvm/runtime/c_backend_api.h"

extern "C" {
    // TVM generated functions
    int32_t tvmgen_default_run(
        int8_t* input_buffer,
        int8_t* output_buffer
    );
}

class TVMModel {
private:
    static const char* TAG;
    
    // Single working buffer (TVM handles memory reuse internally)
    static constexpr size_t WORKSPACE_SIZE = 150000;  // 150KB should be enough
    uint8_t* workspace;
    
public:
    TVMModel() {
        workspace = (uint8_t*)heap_caps_malloc(WORKSPACE_SIZE, MALLOC_CAP_INTERNAL);
        ESP_LOGI(TAG, "Allocated %d KB for TVM inference", WORKSPACE_SIZE / 1024);
    }
    
    ~TVMModel() {
        free(workspace);
    }
    
    bool predict(uint8_t* rgb_image_96x96) {
        // Convert to INT8 channels-first format
        int8_t input_tensor[3 * 96 * 96];
        
        // Rearrange from HWC to CHW and quantize
        for (int c = 0; c < 3; c++) {
            for (int h = 0; h < 96; h++) {
                for (int w = 0; w < 96; w++) {
                    int src_idx = h * 96 * 3 + w * 3 + c;
                    int dst_idx = c * 96 * 96 + h * 96 + w;
                    input_tensor[dst_idx] = (int8_t)(rgb_image_96x96[src_idx] - 128);
                }
            }
        }
        
        // Run TVM inference
        int8_t output;
        int32_t status = tvmgen_default_run(input_tensor, &output);
        
        if (status != 0) {
            ESP_LOGE(TAG, "TVM inference failed with status %d", status);
            return false;
        }
        
        // Interpret output (threshold at 0 for quantized model)
        bool is_human = (output > 0);
        
        ESP_LOGI(TAG, "TVM inference complete. Human: %s (confidence: %d)",
                 is_human ? "YES" : "NO", output);
        
        return is_human;
    }
};

const char* TVMModel::TAG = "TVMModel";
'''
    
    with open(f"{output_dir}/tvm_model_wrapper.cpp", "w") as f:
        f.write(wrapper_code)
    
    # Create CMake file for ESP-IDF
    cmake_code = '''idf_component_register(
    SRCS "tvm_model_wrapper.cpp"
         "codegen/host/src/default_lib0.c"
         "codegen/host/src/default_lib1.c"
         "runtime/src/runtime.c"
    INCLUDE_DIRS "runtime/include" "codegen/host/include"
)
'''
    
    with open(f"{output_dir}/CMakeLists.txt", "w") as f:
        f.write(cmake_code)

def estimate_memory_usage(mod, params):
    """Estimate the memory usage after TVM optimization."""
    
    # Analyze the Relay graph
    print("\n=== Memory Usage Estimation ===")
    
    # Get all tensor shapes in the graph
    shape_dict = {}
    for var in mod["main"].params:
        shape_dict[var.name_hint] = var.type_annotation.shape
    
    # This is a rough estimate - TVM will optimize further
    total_activation_memory = 0
    for name, shape in shape_dict.items():
        size = np.prod([int(dim) for dim in shape])
        print(f"{name}: {shape} = {size} bytes")
        total_activation_memory += size
    
    total_param_memory = sum([np.prod(p.shape) for p in params.values()])
    
    print(f"\nEstimated parameter memory: {total_param_memory / 1024:.1f} KB")
    print(f"Maximum activation memory (before optimization): {total_activation_memory / 1024:.1f} KB")
    print(f"After TVM optimization: likely 50-70% less due to buffer reuse")

if __name__ == "__main__":
    # Install requirements:
    # pip install apache-tvm tflite
    
    print("TVM Compilation for ESP32")
    print("=" * 50)
    
    # Check if TVM is installed
    try:
        print(f"TVM version: {tvm.__version__}")
    except:
        print("Please install TVM first:")
        print("pip install apache-tvm")
        exit(1)
    
    # Compile the model
    tflite_path = "minisqueezenet_96.tflite"
    
    if os.path.exists(tflite_path):
        executor = compile_model_for_esp32(tflite_path)
        print("\nNext steps:")
        print("1. Copy tvm_model/ directory to your ESP32 project")
        print("2. Include tvm_model_wrapper.cpp in your build")
        print("3. Use TVMModel class instead of TensorFlow")
    else:
        print(f"Error: {tflite_path} not found")
        print("Please run the TensorFlow notebook first to generate the quantized model")