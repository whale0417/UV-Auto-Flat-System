import pandas as pd
import os

def multiple_txt_to_excel(input_dir, output_file, file_prefix, file_count):
    """
    将多个文本文件中的数据写入Excel文件的不同Sheet中
    
    参数:
    input_dir (str): 输入的文本文件目录路径
    output_file (str): 输出的Excel文件路径
    file_prefix (str): 文件前缀
    file_count (int): 文件数量
    """
    try:
        # 检查输出目录是否存在，不存在则创建
        output_dir = os.path.dirname(output_file)
        if output_dir and not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        # 创建Excel writer对象
        with pd.ExcelWriter(output_file, engine='openpyxl') as writer:
            for i in range(1, file_count + 1):
                # 构建文件路径
                input_filename = f"{file_prefix}{i}.txt"
                input_filepath = os.path.join(input_dir, input_filename)
                
                # 检查文件是否存在
                if not os.path.exists(input_filepath):
                    print(f"警告: 文件 '{input_filepath}' 不存在，跳过")
                    continue
                
                # 读取文本文件
                df = pd.read_csv(input_filepath, header=None, encoding='utf-8-sig')
                
                # 将数据写入Excel的不同Sheet
                sheet_name = f"b{i}"
                df.to_excel(writer, sheet_name=sheet_name, index=False, header=False)
                
                print(f"文件 '{input_filename}' 的数据已写入Sheet '{sheet_name}'")
        
        print(f"\n所有数据已成功写入到 {output_file}")
        
    except Exception as e:
        print(f"处理过程中出现错误: {str(e)}")

# 主程序
if __name__ == "__main__":
    # 设置参数
    input_dir = r"D:\桌面文件\紫光传感器标定数据\程序\labview36\raw_data"
    output_file = r"D:\桌面文件\out1.xlsx"
    file_prefix = "S_rawdata_b"
    file_count = 9  # 从b1到b9共9个文件
    
    # 执行转换
    multiple_txt_to_excel(input_dir, output_file, file_prefix, file_count)