package decad

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

func brakeFixtureExtrude(t *testing.T, ctx context.Context, doc *Document, circle bool, x0, y0, x1, y1, length float64, frame r3.Frame) *Body {
	t.Helper()
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		t.Fatal(err)
	}
	if circle {
		s.AddConstraint(sketch.NewRadius(s.CreateCircle(s.Origin(), x0), x0))
	} else {
		rect := s.CreateRectangle(x0, y0, x1, y1)
		s.AddConstraint(sketch.NewHorizontalDistance(s.Origin(), rect.A, x0), sketch.NewVerticalDistance(s.Origin(), rect.A, y0), sketch.NewHorizontalDistance(rect.A, rect.B, x1-x0), sketch.NewVerticalDistance(rect.B, rect.C, y1-y0))
	}
	if _, err = s.Solve(ctx); err != nil {
		t.Fatal(err)
	}
	profiles := s.Profiles()
	if len(profiles) != 1 || !profiles[0].Valid {
		t.Fatal("fixture profile invalid")
	}
	body, err := Extrude(ctx, doc, s, profiles[0], Distance{D: units.Millimeters(length), Dir: Along})
	if err != nil {
		t.Fatal(err)
	}
	xf, err := r3.FromFrame(frame)
	if err != nil {
		t.Fatal(err)
	}
	body, err = Transform(ctx, body, xf)
	if err != nil {
		t.Fatal(err)
	}
	return body
}

func brakeFixtureFrame(t *testing.T, origin, u, v r3.Vec) r3.Frame {
	t.Helper()
	f, err := r3.NewFrame(origin, u, v)
	if err != nil {
		t.Fatal(err)
	}
	return f
}
func brakeFixtureBox(t *testing.T, ctx context.Context, doc *Document, x0, x1, y0, y1, z0, z1 float64) *Body {
	t.Helper()
	return brakeFixtureExtrude(t, ctx, doc, false, x0, y0, x1, y1, z1-z0, brakeFixtureFrame(t, r3.Vec{Z: z0}, r3.Vec{X: 1}, r3.Vec{Y: 1}))
}

func brakeGalleryFixture(t *testing.T, ctx context.Context) (*Document, *Body, *Body) {
	t.Helper()
	doc := New()
	right := -140.0 + (42 + 0.6*math.Sqrt(5))
	shaft := brakeFixtureExtrude(t, ctx, doc, true, 2, 0, 0, 0, 34, brakeFixtureFrame(t, r3.Vec{X: right, Y: -75, Z: 600}, r3.Vec{X: 1}, r3.Vec{Z: -1}))
	flat := brakeFixtureBox(t, ctx, doc, right+1.3, right+3, -75, -68, 597, 603)
	target, err := Cut(ctx, shaft, flat)
	if err != nil {
		t.Fatalf("shaft Cut: %v", err)
	}
	blank := brakeFixtureBox(t, ctx, doc, -3.5, 3.5, -6, 2.8, -3, 3)
	slot := brakeFixtureBox(t, ctx, doc, -4, 4, -3, 3.8, -.8, .8)
	pin := brakeFixtureExtrude(t, ctx, doc, true, 1.6, 0, 0, 0, 6, brakeFixtureFrame(t, r3.Vec{Z: -3}, r3.Vec{X: 1}, r3.Vec{Y: 1}))
	tool, err := Union(ctx, slot, pin)
	if err != nil {
		t.Fatalf("clevis tool Union: %v", err)
	}
	c0, err := Cut(ctx, blank, tool)
	if err != nil {
		t.Fatalf("clevis Cut: %v", err)
	}
	cert, ok := c0.n.PrivateShift.(*booleaneval.ShiftCertificate)
	if !ok || !cert.IsPureGrowth() {
		t.Fatalf("clevis Cut did not retain one pure growth: certificate=%T", c0.n.PrivateShift)
	}
	xf, err := r3.FromFrame(brakeFixtureFrame(t, r3.Vec{X: right, Y: -38, Z: 600}, r3.Vec{X: 1}, r3.Vec{Y: 1}))
	if err != nil {
		t.Fatal(err)
	}
	clevis, err := Transform(ctx, c0, xf)
	if err != nil {
		t.Fatal(err)
	}
	return doc, target, clevis
}
