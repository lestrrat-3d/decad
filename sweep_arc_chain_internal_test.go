package decad

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/revolveproof"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// The real Sketch producer and noncardinal arc must charge path-axis error
// in the public mesh bound. Removing that charge makes this bound comparison
// fail even though a coarse 0.1 mm tessellation still renders the shape.
func TestChainArcSweepAxisErrorChangesPublicMeshBound(t *testing.T) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		t.Fatal(err)
	}
	a, b := s.CreatePoint(0, 0), s.CreatePoint(2, 1)
	s.Fix(a)
	s.Fix(b)
	s.CreateLine(a, b)
	if _, err := s.Solve(t.Context()); err != nil {
		t.Fatal(err)
	}
	path, err := NewPath(r3.Vec{}, ArcThrough{Through: r3.NewVec(3, 4, 5), End: r3.NewVec(6, 8, 0)})
	if err != nil {
		t.Fatal(err)
	}
	body, err := Sweep(t.Context(), New(), s, s.Chains()[0], path)
	if err != nil {
		t.Fatal(err)
	}
	mesh, err := body.Tessellate(t.Context(), units.Millimeters(0.1), WithVerification(VerifyAll))
	if err != nil {
		t.Fatal(err)
	}
	sp := body.n.Payload.(chainSweepPayload)
	rp := sp.Revolve.Revolve()
	resolved, err := revolveaxis.ResolveChain(t.Context(), sp.Revolve.Chains[0],
		freeform.NewFreeformWork(), "axis-charge test", rp.ChargedWalk, rp.Ax.SnapTol)
	if err != nil {
		t.Fatal(err)
	}
	plane, held := resolved.Plane[0], resolved.Walks[0]
	zero := rp
	zero.Ax.AUBound, zero.Ax.AVBound, zero.Ax.DUBound, zero.Ax.DVBound = 0, 0, 0, 0
	fullZ0, fullR0, ok := revolveproof.ChainArcSourcePoint(rp.Ax, plane.StartU, plane.StartV, plane.StartBound)
	if !ok {
		t.Fatal("source start has no interval")
	}
	fullZ1, fullR1, ok := revolveproof.ChainArcSourcePoint(rp.Ax, plane.EndU, plane.EndV, plane.EndBound)
	if !ok {
		t.Fatal("source end has no interval")
	}
	_, zeroR0, _ := revolveproof.ChainArcSourcePoint(zero.Ax, plane.StartU, plane.StartV, plane.StartBound)
	_, zeroR1, _ := revolveproof.ChainArcSourcePoint(zero.Ax, plane.EndU, plane.EndV, plane.EndBound)
	phi0, _ := rp.Den.Phi0.Enclosure()
	phi1, _ := rp.Den.Phi1.Enclosure()
	span := proofbound.IntervalSub(phi1, phi0)
	n := len(mesh.n.Triangles) / 2
	angular, err := revolvemesh.AngularSequence(revolvemesh.AngularInput{
		Phi0: rp.Phi0, Phi1: rp.Phi1, Den: rp.Den,
	}, n)
	if err != nil {
		t.Fatal(err)
	}
	fullArea, err := revolveproof.ChainArcAreaDensityAllow(fullZ0, fullR0, fullZ1, fullR1,
		held.StartU, held.StartV, held.EndU, held.EndV, span, angular.Step, n)
	if err != nil {
		t.Fatal(err)
	}
	zeroZ0, _, _ := revolveproof.ChainArcSourcePoint(zero.Ax, plane.StartU, plane.StartV, plane.StartBound)
	zeroZ1, _, _ := revolveproof.ChainArcSourcePoint(zero.Ax, plane.EndU, plane.EndV, plane.EndBound)
	zeroArea, err := revolveproof.ChainArcAreaDensityAllow(zeroZ0, zeroR0, zeroZ1, zeroR1,
		held.StartU, held.StartV, held.EndU, held.EndV, span, angular.Step, n)
	if err != nil {
		t.Fatal(err)
	}
	if !(fullArea > zeroArea) {
		t.Fatalf("path-axis uncertainty must enlarge area allowance: full=%g omitted=%g", fullArea, zeroArea)
	}
	radius := proofbound.RatFloatUp(proofbound.RatMax(zeroR0.Hi, zeroR1.Hi))
	sagWithout := tessellation.ChordSagitta(radius, proofbound.RatFloatUp(span.Hi), n)
	vertexGapWithout := 0.0
	for j, source := range [2]struct {
		u, v  float64
		bound proofbound.WalkEndBound
	}{
		{plane.StartU, plane.StartV, plane.StartBound},
		{plane.EndU, plane.EndV, plane.EndBound},
	} {
		for l := range angular.Samples {
			gap := zero.Lift().SweptPointGap(zero.AxisBound(), zero.Xform,
				source.u, source.v, source.bound, angular.SinIv[l], angular.CosIv[l],
				mesh.n.Vertices[j*(n+1)+l])
			vertexGapWithout = math.Max(vertexGapWithout, gap)
		}
	}
	omittedBound := proofbound.UpRound(proofbound.AbsSumUpper(sagWithout, vertexGapWithout))
	if !(mesh.n.Bound > omittedBound) {
		t.Fatalf("public bound must exceed the axis-free bound: public=%g omitted=%g", mesh.n.Bound, omittedBound)
	}
}
