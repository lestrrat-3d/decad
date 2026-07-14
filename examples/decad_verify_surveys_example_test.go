package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// The opt-in verification surveys answer manufacturability questions with
// proven, Exact readings on analytic bodies: WithMinWallThickness states the
// spec that no wall may be thinner than the tool that has to cut it, and
// WithPullDirection states the direction the part must pull along. A wall
// proven thinner than the tool — or a face provenly opposing the pull — is
// Violating; a cube whose only wall is its own 100 mm slab is Sound.
func Example_decad_verify_surveys() {
	prism := func(doc *decad.Document, w, d, h float64) error {
		ws := sketch.NewWorld()
		s, err := ws.CreateSketch(ws.XY())
		if err != nil {
			return err
		}
		rect := s.CreateRectangle(0, 0, w, d)
		s.Fix(rect.A)
		if _, err := s.Solve(context.Background()); err != nil {
			return err
		}
		_, err = doc.Extrude(s, s.Profiles()[0], decad.Distance{D: units.Millimeters(h), Dir: decad.Along})
		return err
	}

	// A 10×10×0.5 mm plate: its two 10×10 skins oppose across the material,
	// and the ball between them reads the 0.5 mm wall — proven thinner than
	// a 1 mm tool at any coarseness.
	thin := decad.New()
	if err := prism(thin, 10, 10, 0.5); err != nil {
		fmt.Printf("failed to build: %s\n", err)
		return
	}
	report, err := thin.Verify(context.Background(), decad.WithMinWallThickness(units.Millimeters(1)))
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	plate := report.Bodies[0]
	fmt.Printf("plate: %s, wall %s (%s)\n", plate.Status, plate.MinWallThickness.Value, plate.MinWallThickness.Exactness)

	// A 100 mm cube: its edges sit at 90°, past every legal draft
	// allowance, so its only spanning ball is its center's 100 mm slab.
	cube := decad.New()
	if err := prism(cube, 100, 100, 100); err != nil {
		fmt.Printf("failed to build: %s\n", err)
		return
	}
	report, err = cube.Verify(context.Background(), decad.WithMinWallThickness(units.Millimeters(1)))
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	block := report.Bodies[0]
	fmt.Printf("cube: %s, wall %s, trustworthy %v\n", block.Status, block.MinWallThickness.Value, report.Trustworthy())

	// The cube under a +z pull has no undercut: every wall is exactly
	// perpendicular — the pull slides along it — and the empty listing is a
	// proven all-clear. A tilted pull faces two of its planar faces against
	// the motion, and each listed face is a proven undercut.
	report, err = cube.Verify(context.Background(), decad.WithPullDirection(r3.NewVec(0, 0, 1)))
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	fmt.Printf("pull +z: %s, undercuts %d\n", report.Status, len(report.Bodies[0].Undercuts))

	report, err = cube.Verify(context.Background(), decad.WithPullDirection(r3.NewVec(1, 0, 1)))
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	fmt.Printf("pull tilted: %s, undercuts %d\n", report.Status, len(report.Bodies[0].Undercuts))
	// Output:
	// plate: Violating, wall 0.5 mm (Exact)
	// cube: Sound, wall 100 mm, trustworthy true
	// pull +z: Sound, undercuts 0
	// pull tilted: Violating, undercuts 2
}
