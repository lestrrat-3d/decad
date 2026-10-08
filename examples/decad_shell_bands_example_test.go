package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// Removing both caps of a plate with holes leaves one wall band inside the
// outer edge and one band lining each hole: 1 + k separate lumps. Here the
// 100 x 60 plate has a 10 x 20 slot and a radius-8 bore, so a 3 mm inward
// wall is three bands. Their section areas are 6000 - 94*54 for the rim,
// 16*26 - (4 - pi)*9 - 200 for the slot lining (its grown corners are
// radius-3 arcs) and pi*(11^2 - 8^2) for the bore lining, so the volume is
// (1104 + 66*pi) * 20.
func Example_decad_shell_bands() {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 100, 60)
	s.Fix(rect.A)
	s.CreateRectangle(20, 20, 30, 40)
	s.CreateCircle(s.CreatePoint(70, 30), 8)
	if _, err := s.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	var plate *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) == 2 {
			plate = p
		}
	}

	doc := decad.New()
	box, err := doc.Extrude(s, plate, decad.Distance{D: units.Millimeters(20), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}

	// Remove both caps (planar, normal to the z sweep).
	bands, err := box.Shell(context.Background(), decad.Faces(decad.NormalTo(r3.NewVec(0, 0, 1))), units.Millimeters(3))
	if err != nil {
		fmt.Printf("failed to shell: %s\n", err)
		return
	}

	vol, err := bands.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	mm3, err := vol.Value.In(units.CubicMillimeter)
	if err != nil {
		fmt.Printf("failed to convert volume: %s\n", err)
		return
	}
	report, err := doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}

	fmt.Printf("lumps: %d\n", len(bands.Lumps()))
	for i, lump := range bands.Lumps() {
		fmt.Printf("band %d: %d faces\n", i, len(lump.Shells()[0].Faces()))
	}
	fmt.Printf("volume: %.3f mm^3 (%s)\n", mm3, vol.Exactness)
	fmt.Printf("trustworthy: %v\n", report.Passed())
	// Output:
	// lumps: 3
	// band 0: 10 faces
	// band 1: 14 faces
	// band 2: 4 faces
	// volume: 26226.902 mm^3 (Approximate)
	// trustworthy: true
}
