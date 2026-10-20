package revolveplan

import (
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// RimSource admits only the unplaced XY rectangular meridian swept by an exact
// denoted quarter turn about +X, with the one selected edge the radial line at
// the high-X cap and the sweep's zero-angle plane. setback is the fillet radius
// or chamfer setback, which must carry no displacement.
func (rp Payload) RimSource(edges []*topo.Edge, setback, setbackDelta float64) (revolvemesh.RimSource, bool) {
	if len(edges) != 1 || setbackDelta != 0 || rp.Full || rp.SurfaceResult || rp.SectionDelta != 0 ||
		rp.SectionWhole || !rp.RadialProof || rp.Xform != r3.Identity() ||
		len(rp.Profile.Holes) != 0 || len(rp.Profile.Outer.Segments) != 4 ||
		len(rp.BlendSegs) != 0 || rp.BlendKind != "" || rp.DraftCircleCaps || rp.DraftCircleShellRoles {
		return revolvemesh.RimSource{}, false
	}
	if rp.Frame.Origin() != (r3.Vec{}) || rp.Frame.U() != (r3.Vec{X: 1}) ||
		rp.Frame.V() != (r3.Vec{Y: 1}) || rp.Ax.AU != 0 || rp.Ax.AV != 0 ||
		rp.Ax.DU != 1 || rp.Ax.DV != 0 || rp.Ax.AUBound != 0 || rp.Ax.AVBound != 0 ||
		rp.Ax.DUBound != 0 || rp.Ax.DVBound != 0 || !rp.Ax.RadialProof || rp.Phi0 != 0 ||
		rp.Den.Phi0.Span != nil || rp.Den.Phi0.Rad == nil || rp.Den.Phi0.Turn == nil ||
		rp.Den.Phi0.Rad.Sign() != 0 || rp.Den.Phi0.Turn.Sign() != 0 ||
		rp.Den.Phi1.Span != nil || rp.Den.Phi1.Rad == nil || rp.Den.Phi1.Turn == nil ||
		rp.Den.Phi1.Rad.Sign() != 0 || rp.Den.Phi1.Turn.Cmp(big.NewRat(1, 4)) != 0 {
		return revolvemesh.RimSource{}, false
	}
	segs := rp.Profile.Outer.Segments
	lines := [4]sectionrecord.LineSeg{}
	for i, seg := range segs {
		line, ok := seg.(sectionrecord.LineSeg)
		if !ok || line.TStart != 0 || line.TEnd != 1 {
			return revolvemesh.RimSource{}, false
		}
		lines[i] = line
	}
	xlo, xhi, outer := lines[0].Start.U, lines[0].End.U, lines[1].End.V
	want := [4]sectionrecord.LineSeg{
		{Start: sectionrecord.Point2{U: xlo}, End: sectionrecord.Point2{U: xhi}, TEnd: 1},
		{Start: sectionrecord.Point2{U: xhi}, End: sectionrecord.Point2{U: xhi, V: outer}, TEnd: 1},
		{Start: sectionrecord.Point2{U: xhi, V: outer}, End: sectionrecord.Point2{U: xlo, V: outer}, TEnd: 1},
		{Start: sectionrecord.Point2{U: xlo, V: outer}, End: sectionrecord.Point2{U: xlo}, TEnd: 1},
	}
	for i := range lines {
		if lines[i] != want[i] {
			return revolvemesh.RimSource{}, false
		}
	}
	length := xhi - xlo
	if proofbound.IsNonFinite(xlo) || proofbound.IsNonFinite(xhi) || proofbound.IsNonFinite(outer) ||
		proofbound.IsNonFinite(setback) || math.Abs(xlo) > 16 || math.Abs(xhi) > 16 ||
		length < 1 || length > 16 || outer < 1 || outer > 16 ||
		setback < 0.125 || setback > 4 || 2*setback >= length || 2*setback >= outer {
		return revolvemesh.RimSource{}, false
	}
	e := edges[0]
	if _, ok := e.Curve.(surfacegeom.Line3); !ok || e.Start == nil || e.End == nil {
		return revolvemesh.RimSource{}, false
	}
	a, b := e.Start.Position, e.End.Position
	lo, hi := r3.NewVec(xhi, 0, 0), r3.NewVec(xhi, outer, 0)
	if (a != lo || b != hi) && (a != hi || b != lo) {
		return revolvemesh.RimSource{}, false
	}
	return revolvemesh.RimSource{XLo: xlo, XHi: xhi, Radius: outer, Setback: setback}, true
}
