package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

// This file builds one angular-cap radial-rim blend of a source-authenticated
// quarter cylinder. Its x-bound is either the fillet circle or a chamfer line;
// the other curved bound is the source cylinder. Shared z stations give an
// inscribed polyhedron, so no mesh Boolean contact decision is involved.

// tryRevolveCapRimFillet admits only the unplaced XY rectangular meridian
// swept by an exact denoted quarter turn about +X. The selected edge is the
// radial line at the high-X cap and the sweep's zero-angle plane.
func tryRevolveCapRimFillet(ctx context.Context, body *Body, rp revolvePayload,
	edges []*Edge, radius, radiusDelta float64) (*Body, bool, error) {
	source, ok := revolveRimSourceOf(rp, edges, radius, radiusDelta)
	if !ok {
		return nil, false, nil
	}
	out, err := buildRevolveCapRimBlend(ctx, body, source)
	return out, true, err
}

// tryRevolveCapRimChamfer uses the same source class and one-edge selector as
// the fillet. An asymmetric setback changes the trimming plane and is staged.
func tryRevolveCapRimChamfer(ctx context.Context, body *Body, rp revolvePayload,
	edges []*Edge, setback, setbackDelta float64, asym *asymmetricChamfer) (*Body, bool, error) {
	if asym != nil {
		return nil, false, nil
	}
	source, ok := revolveRimSourceOf(rp, edges, setback, setbackDelta)
	if !ok {
		return nil, false, nil
	}
	source.Kind = revolvemesh.RimChamfer
	out, err := buildRevolveCapRimBlend(ctx, body, source)
	return out, true, err
}

func revolveRimSourceOf(rp revolvePayload, edges []*Edge, setback, setbackDelta float64) (revolvemesh.RimSource, bool) {
	if len(edges) != 1 || setbackDelta != 0 || rp.full || rp.surfaceResult || rp.sectionDelta != 0 ||
		rp.sectionWhole || !rp.radialProof || rp.xform != r3.Identity() ||
		len(rp.profile.Holes) != 0 || len(rp.profile.Outer.Segments) != 4 ||
		len(rp.blendSegs) != 0 || rp.blendKind != "" || rp.draftCircleCaps || rp.draftCircleShellRoles {
		return revolvemesh.RimSource{}, false
	}
	if rp.frame.Origin() != (r3.Vec{}) || rp.frame.U() != (r3.Vec{X: 1}) ||
		rp.frame.V() != (r3.Vec{Y: 1}) || rp.ax.AU != 0 || rp.ax.AV != 0 ||
		rp.ax.DU != 1 || rp.ax.DV != 0 || rp.ax.AUBound != 0 || rp.ax.AVBound != 0 ||
		rp.ax.DUBound != 0 || rp.ax.DVBound != 0 || !rp.ax.RadialProof || rp.phi0 != 0 ||
		rp.den.Phi0.Span != nil || rp.den.Phi0.Rad == nil || rp.den.Phi0.Turn == nil ||
		rp.den.Phi0.Rad.Sign() != 0 || rp.den.Phi0.Turn.Sign() != 0 ||
		rp.den.Phi1.Span != nil || rp.den.Phi1.Rad == nil || rp.den.Phi1.Turn == nil ||
		rp.den.Phi1.Rad.Sign() != 0 || rp.den.Phi1.Turn.Cmp(big.NewRat(1, 4)) != 0 {
		return revolvemesh.RimSource{}, false
	}
	segs := rp.profile.Outer.Segments
	lines := [4]lineSeg{}
	for i, seg := range segs {
		line, ok := seg.(lineSeg)
		if !ok || line.TStart != 0 || line.TEnd != 1 {
			return revolvemesh.RimSource{}, false
		}
		lines[i] = line
	}
	xlo, xhi, outer := lines[0].Start.U, lines[0].End.U, lines[1].End.V
	want := [4]lineSeg{
		{Start: planePoint{U: xlo}, End: planePoint{U: xhi}, TEnd: 1},
		{Start: planePoint{U: xhi}, End: planePoint{U: xhi, V: outer}, TEnd: 1},
		{Start: planePoint{U: xhi, V: outer}, End: planePoint{U: xlo, V: outer}, TEnd: 1},
		{Start: planePoint{U: xlo, V: outer}, End: planePoint{U: xlo}, TEnd: 1},
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
	if _, ok := e.n.Curve.(surfacegeom.Line3); !ok || e.n.Start == nil || e.n.End == nil {
		return revolvemesh.RimSource{}, false
	}
	a, b := e.n.Start.Position, e.n.End.Position
	lo, hi := r3.NewVec(xhi, 0, 0), r3.NewVec(xhi, outer, 0)
	if (a != lo || b != hi) && (a != hi || b != lo) {
		return revolvemesh.RimSource{}, false
	}
	return revolvemesh.RimSource{XLo: xlo, XHi: xhi, Radius: outer, Setback: setback}, true
}

func buildRevolveCapRimBlend(ctx context.Context, body *Body, source revolvemesh.RimSource) (*Body, error) {
	stations, err := revolvemesh.RimStations(source)
	if err != nil {
		return nil, err
	}
	bound, missing, err := revolvemesh.RimBounds(source, stations)
	if err != nil {
		return nil, err
	}
	n := len(stations)
	verts := make([]r3.Vec, 0, 4*n-2)
	for i, s := range stations {
		verts = append(verts, r3.NewVec(source.XLo, 0, s.Z), r3.NewVec(s.X, 0, s.Z))
		if i != n-1 {
			verts = append(verts, r3.NewVec(s.X, s.Y, s.Z), r3.NewVec(source.XLo, s.Y, s.Z))
		}
	}
	roles := facesByRole(body)
	roleOrder := [5]string{prismWallRole(0, 1), prismWallRole(0, 2), prismWallRole(0, 3),
		provenance.RoleCapStart, provenance.RoleCapEnd}
	for _, role := range roleOrder {
		if roles[role] == nil {
			return nil, fmt.Errorf("%w: the quarter-turn rim has no source %s face", ErrUnsupported, role)
		}
	}
	groups := make([]facetGroup, 6)
	for i, role := range roleOrder {
		f := roles[role]
		groups[i] = facetGroup{origins: f.originRefs(), planar: f.isPlanar()}
	}
	ref := body.n.Doc.NextProducer
	blendRole := "fillet(0,1)"
	if source.Kind == revolvemesh.RimChamfer {
		blendRole = "chamfer(0,1)"
	}
	groups[5] = facetGroup{origins: []provenance.Ref{{Producer: ref, Role: blendRole}}}
	tris := make([][3]int, 0, 8*n)
	src := make([]int, 0, 8*n)
	add := func(group, a, b, c int) {
		tris = append(tris, [3]int{a, b, c})
		src = append(src, group)
	}
	quad := func(group, a, b, c, d int) {
		add(group, a, b, c)
		add(group, a, c, d)
	}
	quad(3, 0, 3, 2, 1)
	for i := range n - 1 {
		if i%256 == 0 {
			if err := ctx.Err(); err != nil {
				return nil, err
			}
		}
		a, b, c, d := 4*i, 4*i+1, 4*i+2, 4*i+3
		if i == n-2 {
			at, bt := 4*(i+1), 4*(i+1)+1
			quad(4, a, b, bt, at)
			quad(1, d, at, bt, c)
			add(2, a, at, d)
			add(0, b, c, bt)
			continue
		}
		an, bn, cn, dn := 4*(i+1), 4*(i+1)+1, 4*(i+1)+2, 4*(i+1)+3
		quad(4, a, b, bn, an)
		quad(1, d, dn, cn, c)
		quad(2, a, an, dn, d)
		group := 0
		if i < revolvemesh.RimBlendStations {
			group = 5
		}
		quad(group, b, c, cn, bn)
	}
	if err := tessellation.RequireVertexLinks(ctx, len(verts), tris); err != nil {
		return nil, fmt.Errorf("%w: cap rim facets do not close at a vertex: %s", ErrUnsupported, err)
	}
	vertexBound := make([]float64, len(verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	length := new(big.Rat).Sub(proofarith.FloatRat(source.XHi), proofarith.FloatRat(source.XLo))
	radius := proofarith.FloatRat(source.Radius)
	setback := proofarith.FloatRat(source.Setback)
	r2 := new(big.Rat).Mul(radius, radius)
	trueArea := new(big.Rat).Add(new(big.Rat).Mul(big.NewRat(2, 1), r2),
		new(big.Rat).Add(new(big.Rat).Mul(big.NewRat(4, 1), new(big.Rat).Mul(length, radius)),
			new(big.Rat).Mul(big.NewRat(2, 1), new(big.Rat).Mul(setback, radius))))
	areaSlack := math.Max(proofbound.RatFloatUp(trueArea), facetproof.MeshAreaUpper(verts, tris))
	diameter2 := new(big.Rat).Add(new(big.Rat).Mul(length, length), new(big.Rat).Mul(big.NewRat(2, 1), r2))
	dPair := proofbound.RatSqrtUp(diameter2)
	fp := facetedPayload{verts: verts, tris: tris, src: src, groups: groups,
		vertexBound: vertexBound, meshBound: bound, volSymDiff: missing,
		areaSlack: areaSlack, dPair: dPair, xform: r3.Identity()}
	out, err := buildFacetedBody(ctx, documentHandle(body.n.Doc), ref, fp)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	documentHandle(body.n.Doc).commit(out, body)
	return out, nil
}
