
WIDTH = 120
HEIGHT = 48

def make_image(name, value):
    hi = (value >> 8) & 0xFF
    lo = value & 0xFF

    s = ""
    for _ in range(WIDTH * HEIGHT):
        s += f"\\x{hi:02X}\\x{lo:02X}"

    print(f"""
static const struct {{
  uint8_t width;
  uint8_t height;
  uint8_t bytes_per_pixel;
  uint8_t pixel_data[{WIDTH * HEIGHT * 2 + 1}];
}} {name} = {{
  {WIDTH}, {HEIGHT}, 2,
  "{s}"
}};
""")

make_image("img_red",   0xF800)
make_image("img_green", 0x07E0)
make_image("img_blue",  0x001F)

