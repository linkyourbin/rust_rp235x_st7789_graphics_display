#!/usr/bin/env python3
"""
JPG to RAW Converter with Polarity Correction
专为解决黑白颠倒问题设计 - 生成标准和取反两种极性的RAW文件
"""

import sys
import os
from PIL import Image
import glob
import re
import shutil

# =============== CONFIGURATION ===============
# 设置您的屏幕尺寸
SCREEN_WIDTH = 240
SCREEN_HEIGHT = 240

# 选择要测试的颜色模式 (0-7)
# 0-3 = 标准极性 (非取反)
# 4-7 = 取反极性 (解决黑白颠倒)
TEST_MODES = [0, 1, 2, 3, 4, 5, 6, 7]
# =============================================

def convert_to_raw(image_path, output_path, width, height, mode):
    """Convert image to RAW with specific color mode and polarity"""
    img = Image.open(image_path)
    
    # 确保是RGB模式
    if img.mode == 'RGBA':
        background = Image.new('RGB', img.size, (255, 255, 255))
        background.paste(img, mask=img.split()[3])
        img = background
    elif img.mode != 'RGB':
        img = img.convert('RGB')
    
    # 调整大小
    img = img.resize((width, height), Image.LANCZOS)
    
    # 颜色转换
    with open(output_path, "wb") as f:
        for y in range(height):
            for x in range(width):
                r, g, b = img.getpixel((x, y))
                
                # 基础颜色转换
                if mode == 0 or mode == 4:  # 标准 RGB565
                    r5 = (r >> 3) & 0x1F
                    g6 = (g >> 2) & 0x3F
                    b5 = (b >> 3) & 0x1F
                    pixel = (r5 << 11) | (g6 << 5) | b5
                elif mode == 1 or mode == 5:  # 标准 BGR565
                    b5 = (b >> 3) & 0x1F
                    g6 = (g >> 2) & 0x3F
                    r5 = (r >> 3) & 0x1F
                    pixel = (b5 << 11) | (g6 << 5) | r5
                elif mode == 2 or mode == 6:  # 紧凑 RGB565
                    r5 = (r >> 3) & 0x1F
                    g5 = (g >> 3) & 0x1F
                    b5 = (b >> 3) & 0x1F
                    pixel = (r5 << 11) | (g5 << 6) | b5
                elif mode == 3 or mode == 7:  # 紧凑 BGR565
                    b5 = (b >> 3) & 0x1F
                    g5 = (g >> 3) & 0x1F
                    r5 = (r >> 3) & 0x1F
                    pixel = (b5 << 11) | (g5 << 6) | r5
                
                # 极性处理（关键修复：解决黑白颠倒）
                if mode >= 4:  # 取反极性
                    pixel = 0xFFFF - pixel
                
                # 小端序写入
                f.write(bytes([pixel & 0xFF, (pixel >> 8) & 0xFF]))

def generate_test_image():
    """生成测试图片 (红/绿/蓝三色条 + 黑白渐变)"""
    test_img = Image.new('RGB', (SCREEN_WIDTH, SCREEN_HEIGHT))
    
    # 创建三色条 (每80像素一种颜色)
    for y in range(SCREEN_HEIGHT):
        for x in range(SCREEN_WIDTH):
            # 左侧三色条
            if x < 80:
                test_img.putpixel((x, y), (255, 0, 0))  # 红
            elif x < 160:
                test_img.putpixel((x, y), (0, 255, 0))  # 绿
            elif x < 240:
                test_img.putpixel((x, y), (0, 0, 255))  # 蓝
            
            # 右侧黑白渐变 (额外测试黑白)
            if x >= 120:
                gray = int(255 * (x - 120) / (SCREEN_WIDTH - 120))
                test_img.putpixel((x, y), (gray, gray, gray))
    
    # 保存测试图片
    test_path = os.path.join("imgs", "test_pattern.jpg")
    os.makedirs("imgs", exist_ok=True)
    test_img.save(test_path, quality=95)
    print("✅ Generated test pattern: {}".format(test_path))
    return test_path

def auto_detect_directories():
    """自动检测项目结构"""
    current_dir = os.getcwd()
    print("📁 Project directory: {}".format(current_dir))
    
    # 检测 imgs/ 目录
    imgs_dir = os.path.join(current_dir, "imgs")
    if not os.path.exists(imgs_dir):
        print("⚠️ 'imgs' directory not found. Creating with test pattern...")
        os.makedirs(imgs_dir, exist_ok=True)
        return imgs_dir, generate_test_image()
    
    # 检查是否有图片
    image_files = glob.glob(os.path.join(imgs_dir, "*.*"))
    if not image_files:
        print("⚠️ No images in 'imgs' directory. Adding test pattern...")
        return imgs_dir, generate_test_image()
    
    return imgs_dir, None

def main():
    print("🔍 Starting polarity correction for your display...")
    
    # 自动检测目录
    imgs_dir, test_image = auto_detect_directories()
    
    # 获取所有图片
    image_files = (
        glob.glob(os.path.join(imgs_dir, "*.jpg")) +
        glob.glob(os.path.join(imgs_dir, "*.jpeg")) +
        glob.glob(os.path.join(imgs_dir, "*.png"))
    )
    
    if not image_files:
        print("❌ No images found. Aborting.")
        return
    
    print("🖼️ Found {} images to test".format(len(image_files)))
    print("⚙️ Testing {} color modes (0-3=normal, 4-7=inverted): {}".format(len(TEST_MODES), TEST_MODES))
    
    # 为每种模式创建输出目录
    base_assets = "assets"
    shutil.rmtree(base_assets, ignore_errors=True)  # 清除旧数据
    
    # 模式说明
    mode_desc = {
        0: "Standard RGB565 (normal)",
        1: "Standard BGR565 (normal)",
        2: "Compact RGB565 (normal)",
        3: "Compact BGR565 (normal)",
        4: "Standard RGB565 (inverted)",
        5: "Standard BGR565 (inverted)",
        6: "Compact RGB565 (inverted)",
        7: "Compact BGR565 (inverted)"
    }
    
    for mode in TEST_MODES:
        mode_dir = os.path.join(base_assets, "mode_{}".format(mode))
        os.makedirs(mode_dir, exist_ok=True)
        
        print("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        print("🧪 Testing MODE {}: {}".format(mode, mode_desc[mode]))
        print("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━")
        
        # 转换所有图片
        for img_path in image_files:
            filename = os.path.basename(img_path)
            raw_path = os.path.join(mode_dir, os.path.splitext(filename)[0] + ".raw")
            convert_to_raw(img_path, raw_path, SCREEN_WIDTH, SCREEN_HEIGHT, mode)
            print("  ✅ {} → mode_{}/".format(filename, mode))
    
    # 生成使用说明
    readme = "# Polarity Correction Test Results\n\n"
    readme += "Your display likely uses one of these configurations:\n"
    for mode, desc in mode_desc.items():
        readme += "- MODE {}: {}\n".format(mode, desc)
    
    readme += "\n## How to identify correct mode:\n"
    readme += "1. Flash each MODE_* directory to your device\n"
    readme += "2. Look for the test pattern (if generated):\n"
    readme += "   - Left: RED | GREEN | BLUE vertical stripes\n"
    readme += "   - Right: Black → White gradient\n"
    readme += "   - CORRECT MODE:\n"
    readme += "        * Left strip: Pure Red, Green, Blue\n"
    readme += "        * Right gradient: Smooth black to white\n"
    readme += "        * NO black/white inversion\n\n"
    
    readme += "## Rust usage for correct mode:\n"
    readme += "```rust\n"
    readme += "// For normal polarity (MODE 0-3):\n"
    readme += "let img = ImageRawLE::<Rgb565>::new(include_bytes!(\"../assets/mode_X/your_image.raw\"), {});\n\n".format(SCREEN_WIDTH)
    readme += "// For inverted polarity (MODE 4-7):\n"
    readme += "// Option 1: Use inverted RAW files (recommended)\n"
    readme += "let img = ImageRawLE::<Rgb565>::new(include_bytes!(\"../assets/mode_X/your_image.raw\"), {});\n\n".format(SCREEN_WIDTH)
    readme += "// Option 2: Keep normal RAW but invert display (advanced)\n"
    readme += "//   display.command(st7789::Command::INVON, &[])?; // Enable inversion\n"
    readme += "```\n\n"
    
    readme += "## Next steps:\n"
    readme += "1. Identify correct mode from test results\n"
    readme += "2. Delete other mode_* directories\n"
    readme += "3. Rename correct directory to 'assets'\n"
    
    with open(os.path.join(base_assets, "HOW_TO_USE.txt"), "w") as f:
        f.write(readme)
    
    print("\n" + "="*60)
    print("✅ CONVERSION COMPLETE! Next steps:")
    print("1. Check the 'assets' directory for MODE_0 to MODE_7")
    print("2. Flash each mode to your device and find the correct one")
    print("3. Look for: NO black/white inversion in the gradient")
    print("4. See HOW_TO_USE.txt for detailed instructions")
    print("="*60)

if __name__ == "__main__":
    main()