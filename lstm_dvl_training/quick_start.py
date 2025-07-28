"""
LSTM DVL训练快速开始脚本
一键完成数据处理、模型训练和评估
"""

import os
import sys
import argparse
from datetime import datetime

from config import config
from data_processor import DataProcessor
from train_lstm import LSTMTrainer
from evaluate import LSTMEvaluator

def main():
    parser = argparse.ArgumentParser(description='LSTM DVL Training Quick Start')
    parser.add_argument('--skip_data_processing', action='store_true',
                       help='Skip data processing if processed data already exists')
    parser.add_argument('--skip_training', action='store_true',
                       help='Skip training if model already exists')
    parser.add_argument('--model_path', type=str, default=None,
                       help='Path to existing model for evaluation only')
    args = parser.parse_args()
    
    print("=" * 70)
    print("LSTM DVL Training - Quick Start")
    print("Based on: An Underwater Multisource Fusion Anomaly Detection")
    print("Navigation Algorithm Based on Factor Graph and LSTM")
    print("=" * 70)
    
    # 打印配置信息
    config.print_config()
    
    # 步骤1: 数据处理
    if not args.skip_data_processing:
        print("\n" + "="*50)
        print("STEP 1: Data Processing")
        print("="*50)
        
        processor = DataProcessor(config)
        train_data, val_data, test_data = processor.process_all()
        
        print("Data processing completed!")
    else:
        print("\nSkipping data processing...")
    
    # 步骤2: 模型训练
    if not args.skip_training and args.model_path is None:
        print("\n" + "="*50)
        print("STEP 2: Model Training")
        print("="*50)
        
        trainer = LSTMTrainer(config)
        trainer.train()
        
        # 使用最佳模型进行评估
        best_model_path = os.path.join(config.MODEL_DIR, 'best_model.pth')
        
        print("Training completed!")
    else:
        if args.model_path:
            best_model_path = args.model_path
            print(f"\nUsing existing model: {best_model_path}")
        else:
            best_model_path = os.path.join(config.MODEL_DIR, 'best_model.pth')
            print("\nSkipping training...")
    
    # 步骤3: 模型评估
    if os.path.exists(best_model_path):
        print("\n" + "="*50)
        print("STEP 3: Model Evaluation")
        print("="*50)
        
        evaluator = LSTMEvaluator(config, best_model_path)
        results = evaluator.evaluate_model()
        
        print("Evaluation completed!")
        
        # 生成最终报告
        print("\n" + "="*50)
        print("FINAL RESULTS SUMMARY")
        print("="*50)
        
        if 'metrics' in results:
            metrics = results['metrics']
            
            print("Model Performance:")
            if 'overall' in metrics:
                print(f"  Overall RMSE: {metrics['overall']['rmse']:.4f} m")
                print(f"  Overall MAE:  {metrics['overall']['mae']:.4f} m")
            
            if 'horizontal' in metrics:
                print(f"  Horizontal Mean Error: {metrics['horizontal']['mean_error']:.4f} m")
                print(f"  Horizontal 95th Percentile: {metrics['horizontal']['percentile_95']:.4f} m")
            
            print(f"\nModel Info:")
            if 'model_info' in results:
                model_info = results['model_info']
                print(f"  Total Parameters: {model_info['total_parameters']:,}")
                print(f"  LSTM Architecture: {model_info['lstm_units']}")
            
            print(f"\nResults saved to: {config.RESULTS_DIR}")
            
            # 与论文基准对比
            print(f"\nPaper Reference Comparison:")
            print(f"  Paper: 'An Underwater Multisource Fusion Anomaly Detection")
            print(f"         Navigation Algorithm Based on Factor Graph and LSTM'")
            print(f"  Architecture: 3-layer LSTM with units [128, 64, 32]")
            print(f"  Learning Rate: {config.INITIAL_LEARNING_RATE}")
            print(f"  Gradient Clip: {config.GRADIENT_CLIP_VALUE}")
            
    else:
        print(f"\nModel not found: {best_model_path}")
        print("Please run training first or provide a valid model path.")
    
    print("\n" + "="*70)
    print("LSTM DVL Training Pipeline Completed!")
    print("="*70)

if __name__ == "__main__":
    main()
