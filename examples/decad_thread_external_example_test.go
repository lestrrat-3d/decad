package examples_test

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// threadGroove draws the closed V of a 60° thread groove on the XY plane:
// its root at radius root, its mouth at radius mouth, centred at height z0.
// The mouth's half-width is tan 30° times the 1.2 mm the V spans radially.
func threadGroove(root, mouth, z0 float64) (*sketch.Sketch, *sketch.Profile, error) {
	const halfWidth = 0.6928203230275509
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, nil, err
	}
	corners := [][2]float64{{root, z0}, {mouth, z0 - halfWidth}, {mouth, z0 + halfWidth}}
	points := make([]*sketch.Point, len(corners))
	for i, c := range corners {
		points[i] = s.CreatePoint(c[0], c[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	if _, err := s.Solve(context.Background()); err != nil {
		return nil, nil, err
	}
	return s, s.Profiles()[0], nil
}

// clippedGrooveMoment is ∫ρ dA over the part of that V on the root's side of
// radius r: the V cut by ρ = r is a smaller V of the same apex.
func clippedGrooveMoment(root, mouth, r float64) float64 {
	const halfWidth = 0.6928203230275509
	run := math.Abs(r - root)
	t := run / math.Abs(mouth-root)
	return t * halfWidth * run * (root + 2*r) / 3
}

// An external thread is a Cut whose tool is a coil. The groove is a 60° V of
// depth 0.9 below the R = 5 surface, opening 0.3 mm past it, coiled 8 turns
// at a 1.5 mm pitch about the cylinder's own axis, starting 3 mm up so both
// of the coil's caps sit inside the 20 mm cylinder. The coil's held mesh is
// only proven within its δ, near 9e-4 mm, so the boolean meshes the pair at
// that tolerance rather than the finer one the pair's size alone would ask.
//
// The cylinder's interior is invariant under the screw motion, so the cut
// removes the screw sweep of the groove's part inside R: Θ·Q with Θ = 16π
// and Q its moment ∫ρ dA. The published volume interval encloses that closed
// form. The volume reading sits inside Verify's default tolerance, while
// the area and centroid bounds, each composed from the rims' trim
// amplification over their long helical length, do not, so the report reads
// Suspect with the solid proven valid.
func Example_decad_thread_external() {
	w := sketch.NewWorld()
	cs, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := cs.CreateRectangle(0, 0, 5, 20)
	cs.Fix(rect.A)
	cs.Fix(rect.C)
	if _, err := cs.Solve(context.Background()); err != nil {
		fmt.Printf("failed to solve: %s\n", err)
		return
	}
	axis := decad.SketchLine{Start: decad.Point2{U: 0, V: 0}, End: decad.Point2{U: 0, V: 1}}

	doc := decad.New()
	cylinder, err := doc.Revolve(cs, cs.Profiles()[0], axis, decad.FullRevolution{})
	if err != nil {
		fmt.Printf("failed to revolve: %s\n", err)
		return
	}
	gs, gp, err := threadGroove(4.1, 5.3, 3)
	if err != nil {
		fmt.Printf("failed to draw the groove: %s\n", err)
		return
	}
	tool, err := doc.Coil(context.Background(), gs, gp, axis, units.Millimeters(1.5), units.Scalar(8))
	if err != nil {
		fmt.Printf("failed to coil: %s\n", err)
		return
	}
	threaded, err := decad.Cut(context.Background(), cylinder, tool)
	if err != nil {
		fmt.Printf("failed to cut: %s\n", err)
		return
	}

	vol, err := threaded.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	report, err := doc.Verify(context.Background())
	if err != nil {
		fmt.Printf("failed to verify: %s\n", err)
		return
	}
	exact := math.Pi * (25*20 - 16*clippedGrooveMoment(4.1, 5.3, 5))
	value, bound := vol.Value.Base(), vol.Bound.Base()
	fmt.Printf("closed form: %.2f mm³\n", exact)
	fmt.Printf("volume: %.1f ± %.1f mm³, encloses the closed form: %v\n", value, bound, math.Abs(value-exact) <= bound)
	fmt.Printf("lumps: %d, validity: %s, verify: %s\n", len(threaded.Lumps()), report.Bodies[0].Validity.Outcome, report.Status)
	// Output:
	// closed form: 1460.31 mm³
	// volume: 1460.0 ± 1.5 mm³, encloses the closed form: true
	// lumps: 1, validity: valid, verify: Suspect
}
