#!/usr/bin/env python3
"""
Tiny CNN model designed specifically for ESP32 memory constraints.
Target: <50KB RAM usage with TFLite Micro
"""

import tensorflow as tf
from tensorflow import keras
from keras import layers

def create_tiny_cnn(input_shape=(96, 96, 3)):
    """
    Ultra-lightweight CNN for human detection on ESP32.
    
    Memory calculation (INT8 quantized):
    - Conv1: 8 filters, 5x5 -> (8 × 92 × 92) = 67KB
    - Pool1: stride 4 -> (8 × 23 × 23) = 4KB  
    - Conv2: 16 filters, 3x3 -> (16 × 21 × 21) = 7KB
    - Pool2: stride 3 -> (16 × 7 × 7) = 784 bytes
    - Dense: 2 outputs
    
    Total intermediate memory: ~80KB (fits in ESP32!)
    """
    
    model = keras.Sequential([
        # Input
        layers.Input(shape=input_shape),
        
        # Conv block 1 - minimal filters
        layers.Conv2D(8, (5, 5), activation='relu', padding='valid'),
        layers.MaxPooling2D(pool_size=(4, 4)),  # Aggressive pooling
        
        # Conv block 2 - still small
        layers.Conv2D(16, (3, 3), activation='relu', padding='valid'),
        layers.MaxPooling2D(pool_size=(3, 3)),
        
        # Classification head
        layers.Flatten(),
        layers.Dense(16, activation='relu'),
        layers.Dropout(0.5),
        layers.Dense(1, activation='sigmoid')  # Binary classification
    ])
    
    return model

def quantize_model(model, representative_dataset):
    """Convert to INT8 quantized TFLite model."""
    
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    
    # Full INT8 quantization
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    converter.representative_dataset = representative_dataset
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type = tf.int8
    converter.inference_output_type = tf.int8
    
    return converter.convert()

if __name__ == "__main__":
    # Create and display model
    model = create_tiny_cnn()
    model.summary()
    
    # Calculate approximate memory usage
    total_params = model.count_params()
    print(f"\nModel size (float32): {total_params * 4 / 1024:.1f} KB")
    print(f"Model size (INT8): {total_params / 1024:.1f} KB")
    
    # Compile for training
    model.compile(
        optimizer='adam',
        loss='binary_crossentropy',
        metrics=['accuracy']
    )
    
    print("\nThis model should use <100KB RAM on ESP32 with TFLite Micro!")