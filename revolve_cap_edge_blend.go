package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/r3"
)

// This file builds one angular-cap radial-rim blend of a source-authenticated
// quarter cylinder. Its x-bound is either the fillet circle or a chamfer line;
// the other curved bound is the source cylinder. Shared z stations give an
// inscribed polyhedron, so no mesh Boolean contact decision is involved.
const (
	revolveRimBlendStations = 128
	revolveRimWallStations  = 512
)

type revolveRimBlendKind uint8

const (
	revolveRimFillet revolveRimBlendKind = iota
	revolveRimChamfer
)

type revolveRimSource struct {
	xlo, xhi, radius, setback float64
	kind                      revolveRimBlendKind
}

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
	source.kind = revolveRimChamfer
	out, err := buildRevolveCapRimBlend(ctx, body, source)
	return out, true, err
}

func revolveRimSourceOf(rp revolvePayload, edges []*Edge, setback, setbackDelta float64) (revolveRimSource, bool) {
	if len(edges) != 1 || setbackDelta != 0 || rp.full || rp.surfaceResult || rp.sectionDelta != 0 ||
		rp.sectionWhole || !rp.radialProof || rp.xform != r3.Identity() ||
		len(rp.profile.Holes) != 0 || len(rp.profile.Outer.Segments) != 4 ||
		len(rp.blendSegs) != 0 || rp.blendKind != "" || rp.draftCircleCaps || rp.draftCircleShellRoles {
		return revolveRimSource{}, false
	}
	if rp.frame.Origin() != (r3.Vec{}) || rp.frame.U() != (r3.Vec{X: 1}) ||
		rp.frame.V() != (r3.Vec{Y: 1}) || rp.ax.AU != 0 || rp.ax.AV != 0 ||
		rp.ax.DU != 1 || rp.ax.DV != 0 || rp.ax.AUBound != 0 || rp.ax.AVBound != 0 ||
		rp.ax.DUBound != 0 || rp.ax.DVBound != 0 || !rp.ax.RadialProof || rp.phi0 != 0 ||
		rp.den.Phi0.Span != nil || rp.den.Phi0.Rad == nil || rp.den.Phi0.Turn == nil ||
		rp.den.Phi0.Rad.Sign() != 0 || rp.den.Phi0.Turn.Sign() != 0 ||
		rp.den.Phi1.Span != nil || rp.den.Phi1.Rad == nil || rp.den.Phi1.Turn == nil ||
		rp.den.Phi1.Rad.Sign() != 0 || rp.den.Phi1.Turn.Cmp(big.NewRat(1, 4)) != 0 {
		return revolveRimSource{}, false
	}
	segs := rp.profile.Outer.Segments
	lines := [4]lineSeg{}
	for i, seg := range segs {
		line, ok := seg.(lineSeg)
		if !ok || line.TStart != 0 || line.TEnd != 1 {
			return revolveRimSource{}, false
		}
		lines[i] = line
	}
	xlo, xhi, outer := lines[0].Start.U, lines[0].End.U, lines[1].End.V
	want := [4]lineSeg{
		{Start: Point2{U: xlo}, End: Point2{U: xhi}, TEnd: 1},
		{Start: Point2{U: xhi}, End: Point2{U: xhi, V: outer}, TEnd: 1},
		{Start: Point2{U: xhi, V: outer}, End: Point2{U: xlo, V: outer}, TEnd: 1},
		{Start: Point2{U: xlo, V: outer}, End: Point2{U: xlo}, TEnd: 1},
	}
	for i := range lines {
		if lines[i] != want[i] {
			return revolveRimSource{}, false
		}
	}
	length := xhi - xlo
	if proofbound.IsNonFinite(xlo) || proofbound.IsNonFinite(xhi) || proofbound.IsNonFinite(outer) ||
		proofbound.IsNonFinite(setback) || math.Abs(xlo) > 16 || math.Abs(xhi) > 16 ||
		length < 1 || length > 16 || outer < 1 || outer > 16 ||
		setback < 0.125 || setback > 4 || 2*setback >= length || 2*setback >= outer {
		return revolveRimSource{}, false
	}
	e := edges[0]
	if _, ok := e.curve.(Line3); !ok || e.start == nil || e.end == nil {
		return revolveRimSource{}, false
	}
	a, b := e.start.position, e.end.position
	lo, hi := r3.NewVec(xhi, 0, 0), r3.NewVec(xhi, outer, 0)
	if (a != lo || b != hi) && (a != hi || b != lo) {
		return revolveRimSource{}, false
	}
	return revolveRimSource{xlo: xlo, xhi: xhi, radius: outer, setback: setback}, true
}

type revolveRimStation struct {
	z, x, y float64
	xUpper  *big.Rat
	yUpper  *big.Rat
}

func revolveRimStations(source revolveRimSource) ([]revolveRimStation, error) {
	n := revolveRimBlendStations + revolveRimWallStations + 1
	stations := make([]revolveRimStation, 0, n)
	for i := 0; i <= revolveRimBlendStations; i++ {
		q := float64(i) / revolveRimBlendStations
		z := source.setback * q * q
		if i == revolveRimBlendStations {
			z = source.setback
		}
		station, ok := revolveRimStationAt(source, z)
		if !ok {
			return nil, fmt.Errorf("%w: the cap rim has no certified blend station", ErrUnsupported)
		}
		stations = append(stations, station)
	}
	for i := 1; i <= revolveRimWallStations; i++ {
		q := float64(revolveRimWallStations-i) / revolveRimWallStations
		z := source.radius - (source.radius-source.setback)*q*q
		if i == revolveRimWallStations {
			z = source.radius
		}
		station, ok := revolveRimStationAt(source, z)
		if !ok {
			return nil, fmt.Errorf("%w: the outer circle has no certified station", ErrUnsupported)
		}
		stations = append(stations, station)
	}
	for i := 1; i < len(stations); i++ {
		if stations[i].z <= stations[i-1].z || stations[i].x < stations[i-1].x ||
			stations[i].y > stations[i-1].y {
			return nil, fmt.Errorf("%w: rounded rim stations are not ordered", ErrUnsupported)
		}
	}
	return stations, nil
}

// The float station z is read as an exact dyadic coordinate. Directed square
// roots place the fillet's x and both blends' y on or inside their analytic
// circles; the chamfer's x is rounded down from its exact affine value. This
// makes every interpolated station rectangle a subset of the true result:
// both upper-bound curves are concave in z.
func revolveRimStationAt(source revolveRimSource, z float64) (revolveRimStation, bool) {
	qz := proofarith.FloatRat(z)
	if qz == nil || z < 0 || z > source.radius {
		return revolveRimStation{}, false
	}
	r := proofarith.FloatRat(source.radius)
	qh := new(big.Rat).Sub(new(big.Rat).Mul(r, r), new(big.Rat).Mul(qz, qz))
	if qh.Sign() < 0 {
		return revolveRimStation{}, false
	}
	y, yu := proofbound.RatSqrtDown(qh), proofbound.RatSqrtUp(qh)
	station := revolveRimStation{z: z, y: y, yUpper: proofarith.FloatRat(yu)}
	if z >= source.setback {
		station.x = source.xhi
		station.xUpper = proofarith.FloatRat(source.xhi)
		return station, true
	}
	qf := proofarith.FloatRat(source.setback)
	if source.kind == revolveRimChamfer {
		x := new(big.Rat).Add(new(big.Rat).Sub(proofarith.FloatRat(source.xhi), qf), qz)
		station.x = proofbound.RatFloatDown(x)
		station.xUpper = x
		return station, !proofbound.IsNonFinite(station.x) && !proofbound.IsNonFinite(station.y)
	}
	qroot := new(big.Rat).Sub(new(big.Rat).Mul(new(big.Rat).Mul(big.NewRat(2, 1), qf), qz),
		new(big.Rat).Mul(qz, qz))
	if qroot.Sign() < 0 {
		return revolveRimStation{}, false
	}
	base := new(big.Rat).Sub(proofarith.FloatRat(source.xhi), qf)
	lo := new(big.Rat).Add(base, proofarith.FloatRat(proofbound.RatSqrtDown(qroot)))
	hi := new(big.Rat).Add(base, proofarith.FloatRat(proofbound.RatSqrtUp(qroot)))
	station.x = proofbound.RatFloatDown(lo)
	station.xUpper = hi
	return station, !proofbound.IsNonFinite(station.x) && !proofbound.IsNonFinite(station.y)
}

func revolveRimBounds(source revolveRimSource, stations []revolveRimStation) (float64, float64, error) {
	r := proofarith.FloatRat(source.radius)
	length := new(big.Rat).Sub(proofarith.FloatRat(source.xhi), proofarith.FloatRat(source.xlo))
	var maxGap, volumeGap *big.Rat
	maxGap, volumeGap = new(big.Rat), new(big.Rat)
	for i := range len(stations) - 1 {
		a, b := stations[i], stations[i+1]
		df := new(big.Rat).Sub(b.xUpper, proofarith.FloatRat(a.x))
		dh := new(big.Rat).Sub(a.yUpper, proofarith.FloatRat(b.y))
		if df.Sign() < 0 || dh.Sign() < 0 {
			return 0, 0, fmt.Errorf("%w: cap rim intervals are not monotone", ErrUnsupported)
		}
		gap := new(big.Rat).Add(df, dh)
		if gap.Cmp(maxGap) > 0 {
			maxGap = gap
		}
		dz := new(big.Rat).Sub(proofarith.FloatRat(b.z), proofarith.FloatRat(a.z))
		density := new(big.Rat).Add(new(big.Rat).Mul(r, df), new(big.Rat).Mul(length, dh))
		volumeGap.Add(volumeGap, new(big.Rat).Mul(dz, density))
	}
	bound, missing := proofbound.RatFloatUp(maxGap), proofbound.RatFloatUp(volumeGap)
	if proofbound.IsNonFinite(bound) || proofbound.IsNonFinite(missing) || bound >= 0.1 {
		return 0, 0, fmt.Errorf("%w: the rim's certified boundary bound exceeds 0.1 mm", ErrUnsupported)
	}
	return bound, missing, nil
}

func buildRevolveCapRimBlend(ctx context.Context, body *Body, source revolveRimSource) (*Body, error) {
	stations, err := revolveRimStations(source)
	if err != nil {
		return nil, err
	}
	bound, missing, err := revolveRimBounds(source, stations)
	if err != nil {
		return nil, err
	}
	n := len(stations)
	verts := make([]r3.Vec, 0, 4*n-2)
	for i, s := range stations {
		verts = append(verts, r3.NewVec(source.xlo, 0, s.z), r3.NewVec(s.x, 0, s.z))
		if i != n-1 {
			verts = append(verts, r3.NewVec(s.x, s.y, s.z), r3.NewVec(source.xlo, s.y, s.z))
		}
	}
	roles := facesByRole(body)
	roleOrder := [5]string{prismWallRole(0, 1), prismWallRole(0, 2), prismWallRole(0, 3),
		roleCapStart, roleCapEnd}
	for _, role := range roleOrder {
		if roles[role] == nil {
			return nil, fmt.Errorf("%w: the quarter-turn rim has no source %s face", ErrUnsupported, role)
		}
	}
	groups := make([]facetGroup, 6)
	for i, role := range roleOrder {
		f := roles[role]
		groups[i] = facetGroup{origins: f.Origins(), planar: f.isPlanar()}
	}
	ref := body.doc.nextProducerID()
	blendRole := "fillet(0,1)"
	if source.kind == revolveRimChamfer {
		blendRole = "chamfer(0,1)"
	}
	groups[5] = facetGroup{origins: []FeatureRef{{producer: ref, Role: blendRole}}}
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
		if i < revolveRimBlendStations {
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
	length := new(big.Rat).Sub(proofarith.FloatRat(source.xhi), proofarith.FloatRat(source.xlo))
	radius := proofarith.FloatRat(source.radius)
	setback := proofarith.FloatRat(source.setback)
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
	out, err := buildFacetedBody(ctx, body.doc, ref, fp)
	if err != nil {
		return nil, err
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	body.doc.commit(out, body)
	return out, nil
}
