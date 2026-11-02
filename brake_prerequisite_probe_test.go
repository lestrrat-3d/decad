package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

func TestBrakePrerequisiteProbe(t *testing.T) {
	ctx := t.Context()
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

	shaftCert, ok := target.n.PrivateShift.(*booleaneval.ShiftCertificate)
	if !ok || !shaftCert.IsTranslation() {
		t.Fatalf("shaft Cut certificate missing or not a translation")
	}
	for _, body := range []*Body{target, c0} {
		mesh, err := body.Tessellate(ctx, units.Millimeters(.12))
		if err != nil || !mesh.BoundaryVerified() || !mesh.VolumeVerified() {
			t.Fatalf("source geometry proof: %v", err)
		}
	}
	cert, ok := c0.n.PrivateShift.(*booleaneval.ShiftCertificate)
	if !ok || !cert.IsPureGrowth() {
		t.Fatalf("clevis Cut certificate missing or not growth")
	}
}
