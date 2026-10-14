package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// The published symmetric-difference allowance must cover the observed
// signed-volume gap and remain smaller than the whole true body. The latter
// rejects a vacuous certificate that permits the entire solid to disappear.
func TestCompositeTwistOccupiedVolumeProof(t *testing.T) {
	world := sketch.NewWorld()
	s, err := world.CreateSketch(world.XY())
	if err != nil {
		t.Fatal(err)
	}
	rect := s.CreateRectangle(-0.1, -0.1, 0.1, 0.1)
	s.Fix(rect.A)
	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatal(err)
	}
	path, err := NewPath(r3.Vec{},
		LineTo{End: r3.NewVec(0, 0, 1)},
		ArcThrough{Through: r3.NewVec(2, 0, 5), End: r3.NewVec(5, 0, 6)})
	if err != nil {
		t.Fatal(err)
	}
	body, err := Sweep(t.Context(), New(), s, s.Profiles()[0], path,
		WithSweepTwist(units.Degrees(5)))
	if err != nil {
		t.Fatal(err)
	}
	payload, ok := body.n.Payload.(compositeTwistedSweepPayload)
	if !ok {
		t.Fatalf("got payload %T", body.n.Payload)
	}
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.5), WithVerification(VerifyAll))
	if err != nil {
		t.Fatal(err)
	}
	volume, err := body.Volume()
	if err != nil {
		t.Fatal(err)
	}
	if payload.mesh.n.VolSymDiff >= volume.Value.Base()-volume.Bound.Base() {
		t.Fatalf("occupied-volume allowance %g covers the whole body %g",
			payload.mesh.n.VolSymDiff, volume.Value.Base())
	}
	heldVolume, heldArea := 0.0, 0.0
	for _, triangle := range mesh.n.Triangles {
		a, b, c := mesh.n.Vertices[triangle[0]], mesh.n.Vertices[triangle[1]], mesh.n.Vertices[triangle[2]]
		heldVolume += a.Dot(b.Cross(c)) / 6
		heldArea += b.Sub(a).Cross(c.Sub(a)).Len() / 2
	}
	if gap := math.Abs(heldVolume - volume.Value.Base()); gap > payload.mesh.n.VolSymDiff+volume.Bound.Base() {
		t.Fatalf("held volume gap %g exceeds occupied-volume allowance %g", gap, payload.mesh.n.VolSymDiff)
	}
	area, err := body.Area()
	if err != nil {
		t.Fatal(err)
	}
	if gap := math.Abs(heldArea - area.Value.Base()); gap > mesh.n.AreaSlack+area.Bound.Base() {
		t.Fatalf("held area gap %g exceeds allowance %g", gap, mesh.n.AreaSlack+area.Bound.Base())
	}
	withoutProof, err := body.Tessellate(t.Context(), units.Millimeters(0.5), WithVerification(VerifyNone))
	if err != nil {
		t.Fatal(err)
	}
	if withoutProof.VolumeVerified() || !mesh.VolumeVerified() {
		t.Fatal("a lower verification request changed the previously certified mesh")
	}
}
