package main

import (
	"fmt"
	"image"
	"image/color"
	"image/png"
	"os"
)

func main() {
	if err := run(); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

func run() error {
	if len(os.Args) != 3 {
		return fmt.Errorf("need source and preview paths")
	}
	in, err := os.Open(os.Args[1])
	if err != nil {
		return err
	}
	defer in.Close()
	src, err := png.Decode(in)
	if err != nil {
		return err
	}
	if src.Bounds() != image.Rect(0, 0, 1280, 960) {
		return fmt.Errorf("unexpected render bounds: %v", src.Bounds())
	}
	dst := image.NewNRGBA(image.Rect(0, 0, 320, 240))
	for y := 0; y < 240; y++ {
		for x := 0; x < 320; x++ {
			var r, g, b, a uint32
			for dy := 0; dy < 4; dy++ {
				for dx := 0; dx < 4; dx++ {
					c := color.NRGBAModel.Convert(src.At(x*4+dx, y*4+dy)).(color.NRGBA)
					r += uint32(c.R)
					g += uint32(c.G)
					b += uint32(c.B)
					a += uint32(c.A)
				}
			}
			dst.SetNRGBA(x, y, color.NRGBA{R: uint8(r / 16), G: uint8(g / 16), B: uint8(b / 16), A: uint8(a / 16)})
		}
	}
	f, err := os.Create(os.Args[2])
	if err != nil {
		return err
	}
	if err := png.Encode(f, dst); err != nil {
		f.Close()
		return err
	}
	return f.Close()
}
