package decad

import (
	"errors"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/obliqueboss"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

func buildObliqueBossSources(t *testing.T, center, tiltAxis r3.Vec) (*Body, *Body) {
	t.Helper()
	ctx := t.Context()
	doc := New()
	w := sketch.NewWorld()
	plateSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		t.Fatal(err)
	}
	rect := plateSketch.CreateRectangle(0, 0, 20, 20)
	plateSketch.Fix(rect.A)
	if _, err := plateSketch.Solve(ctx); err != nil {
		t.Fatal(err)
	}
	plate, err := Extrude(ctx, doc, plateSketch, plateSketch.Profiles()[0],
		Distance{D: units.Millimeters(5), Dir: Along})
	if err != nil {
		t.Fatal(err)
	}
	bossSketch, err := w.CreateSketch(w.XY())
	if err != nil {
		t.Fatal(err)
	}
	p := bossSketch.CreatePoint(0, 0)
	bossSketch.Fix(p)
	bossSketch.CreateCircle(p, 4)
	if _, err := bossSketch.Solve(ctx); err != nil {
		t.Fatal(err)
	}
	boss, err := Extrude(ctx, doc, bossSketch, bossSketch.Profiles()[0],
		Distance{D: units.Millimeters(6), Dir: Along})
	if err != nil {
		t.Fatal(err)
	}
	turn, err := r3.RotationAround(r3.Vec{}, tiltAxis, units.Degrees(15))
	if err != nil {
		t.Fatal(err)
	}
	shift, err := r3.Translation(center)
	if err != nil {
		t.Fatal(err)
	}
	pose, err := turn.Then(shift)
	if err != nil {
		t.Fatal(err)
	}
	boss, err = Transform(ctx, boss, pose)
	if err != nil {
		t.Fatal(err)
	}
	return plate, boss
}

func TestObliqueBossJunctionFilletPublicWorkflow(t *testing.T) {
	testObliqueBossJunctionFilletPublicWorkflow(t, r3.Vec{Y: 1})
}

func TestObliqueBossJunctionFilletXTiltPublicWorkflow(t *testing.T) {
	testObliqueBossJunctionFilletPublicWorkflow(t, r3.Vec{X: 1})
}

func TestObliqueBossJunctionFilletDiagonalTiltPublicWorkflow(t *testing.T) {
	testObliqueBossJunctionFilletPublicWorkflow(t, r3.Vec{X: 1, Y: 1})
}

func testObliqueBossJunctionFilletPublicWorkflow(t *testing.T, tiltAxis r3.Vec) {
	t.Helper()
	ctx := t.Context()
	plate, boss := buildObliqueBossSources(t, r3.Vec{X: 10, Y: 10, Z: 3.5}, tiltAxis)
	joined, err := Union(ctx, plate, boss)
	if err != nil {
		t.Fatal(err)
	}
	joinedMesh, err := joined.Tessellate(ctx, units.Millimeters(0.1), WithVerification(VerifyAll))
	if err != nil || !joinedMesh.BoundaryVerified() || !joinedMesh.VolumeVerified() {
		t.Fatalf("joined mesh proof: %v, boundary=%v, volume=%v", err,
			joinedMesh != nil && joinedMesh.BoundaryVerified(),
			joinedMesh != nil && joinedMesh.VolumeVerified())
	}
	var junction *Edge
	for _, edge := range joined.Edges() {
		if edge.Start() != edge.End() || math.Abs(edge.Start().Position().Value.Z-5) > 1e-9 {
			continue
		}
		// The junction is a Boolean rim: a chord chain, or the ellipse a
		// retained plane and cylinder publish (retention §10.5).
		if _, ok := edge.n.Curve.(surfacegeom.FacetedCurve); !ok && !edge.n.RetainedCurve {
			continue
		}
		if junction != nil {
			t.Fatal("more than one complete top-plane junction edge")
		}
		junction = edge
	}
	if junction == nil {
		t.Fatal("the Union did not produce a complete junction edge")
	}
	query := Edges(EndpointAt(junction.Start().Position().Value)).Exactly(1)
	if selected, err := query.SelectEdges(joined); err != nil || len(selected) != 1 {
		t.Fatalf("junction query selected %d edges: %v", len(selected), err)
	}
	if _, err := Fillet(ctx, joined, query, units.Millimeters(1.1)); !errors.Is(err, ErrUnsupported) {
		t.Fatalf("oversize junction fillet: got %v, want ErrUnsupported", err)
	}
	before, err := joined.Volume()
	if err != nil {
		t.Fatal(err)
	}
	filleted, err := Fillet(ctx, joined, query, units.Millimeters(0.5))
	if err != nil {
		t.Fatal(err)
	}
	after, err := filleted.Volume()
	if err != nil {
		t.Fatal(err)
	}
	if after.Value.Base() <= before.Value.Base() {
		t.Fatalf("the boss-root fillet did not add material: before %g, after %g",
			before.Value.Base(), after.Value.Base())
	}
	centroid, err := filleted.Centroid()
	if err != nil || !(centroid.Bound.Base() > 0) {
		t.Fatalf("the fillet has no positive centroid error bound: %v, %g", err, centroid.Bound.Base())
	}
	mesh, err := filleted.Tessellate(ctx, units.Millimeters(0.1), WithVerification(VerifyAll))
	if err != nil {
		t.Fatal(err)
	}
	if !mesh.BoundaryVerified() || !mesh.VolumeVerified() || mesh.Bound().Base() >= 0.1 {
		t.Fatalf("fillet mesh proof: boundary=%v volume=%v bound=%g",
			mesh.BoundaryVerified(), mesh.VolumeVerified(), mesh.Bound().Base())
	}
}

func TestObliqueBossRejectsPartialJunctionSource(t *testing.T) {
	plate, boss := buildObliqueBossSources(t, r3.Vec{X: 18, Y: 10, Z: 3.5}, r3.Vec{Y: 1})
	if record := obliqueboss.Recognize(bodyNode(plate), bodyNode(boss)); record != nil {
		t.Fatal("a boss intersecting the plate side received a complete-junction record")
	}
}
