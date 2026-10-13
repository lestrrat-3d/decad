package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

const twistShellAxialCells = 8

type twistShellHeld struct {
	verts       []r3.Vec
	tris        [][3]int
	src         []int
	vertexError float64
	outerBound  float64
	innerBound  float64
	capBound    float64
	volumeBound float64
	areaSlack   float64
}

func twistShellDistanceUpper(a, b r3.Vec) float64 {
	dx := new(big.Rat).Sub(twistRat(a.X), twistRat(b.X))
	dy := new(big.Rat).Sub(twistRat(a.Y), twistRat(b.Y))
	dz := new(big.Rat).Sub(twistRat(a.Z), twistRat(b.Z))
	squared := new(big.Rat).Add(new(big.Rat).Add(
		new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy)), new(big.Rat).Mul(dz, dz))
	return proofbound.RatSqrtUp(squared)
}

func twistShellContourBounds(points []Point2) (float64, float64) {
	var radius, edge float64
	for i, p := range points {
		q := points[(i+1)%len(points)]
		radiusRat := new(big.Rat).Add(new(big.Rat).Abs(twistRat(p.U)),
			new(big.Rat).Abs(twistRat(p.V)))
		edgeRat := new(big.Rat).Add(
			new(big.Rat).Abs(new(big.Rat).Sub(twistRat(q.U), twistRat(p.U))),
			new(big.Rat).Abs(new(big.Rat).Sub(twistRat(q.V), twistRat(p.V))))
		radius = max(radius, proofbound.RatFloatUp(radiusRat))
		edge = max(edge, proofbound.RatFloatUp(edgeRat))
	}
	return radius, edge
}

func twistShellRingVertex(p Point2, sourceError, z float64, exactZ *big.Rat,
	angle proofbound.RatInterval, heldAngle float64) (r3.Vec, float64, error) {
	sinIv, cosIv, ok := proofbound.RadSinCosSpan(angle)
	if !ok {
		return r3.Vec{}, 0, fmt.Errorf(`%w: twisted shell ring angle is not bounded`, ErrUnsupported)
	}
	sin, cos := math.Sincos(heldAngle)
	v := r3.NewVec(cos*p.U-sin*p.V, sin*p.U+cos*p.V, z)
	px, py := twistOffsetPoint(p.U), twistOffsetPoint(p.V)
	x := proofbound.IntervalSub(proofbound.IntervalMul(cosIv, px), proofbound.IntervalMul(sinIv, py))
	y := proofbound.IntervalAdd(proofbound.IntervalMul(sinIv, px), proofbound.IntervalMul(cosIv, py))
	e := proofbound.Radius3D(max(proofbound.IntervalFloatError(x, v.X),
		proofbound.IntervalFloatError(y, v.Y),
		proofbound.IntervalFloatError(proofbound.PointInterval(exactZ), v.Z)))
	return v, proofbound.AbsSumUpper(e, sourceError), nil
}

func twistShellSampledMesh(ctx context.Context, stations twistOffsetStations,
	height, thickness float64, angle units.Value,
	outerArea proofbound.RatInterval) (twistShellHeld, error) {
	if err := ctx.Err(); err != nil {
		return twistShellHeld{}, err
	}
	if !twistOffsetConvexPolygon(stations.outer) || !twistOffsetConvexPolygon(stations.inner) ||
		len(stations.outer) != len(stations.inner) || height <= thickness || thickness <= 0 {
		return twistShellHeld{}, fmt.Errorf(`%w: twisted shell has no paired convex contours`, ErrUnsupported)
	}
	thetaIv, ok := revolveangle.FromValue(angle).Enclosure()
	theta, err := angle.In(units.Radian)
	if !ok || err != nil {
		return twistShellHeld{}, fmt.Errorf(`%w: twisted shell has no bounded turn`, ErrUnsupported)
	}
	n := len(stations.outer)
	outerRadius, outerEdge := twistShellContourBounds(stations.outer)
	innerRadius, innerEdge := twistShellContourBounds(stations.inner)
	var outerPointError, innerPointError float64
	for i := range stations.outer {
		outerPointError = max(outerPointError, stations.outerVertex[i])
		innerPointError = max(innerPointError, stations.innerVertex[i])
	}
	outerRadius = proofbound.AbsSumUpper(outerRadius,
		proofbound.ProductUpper(2, stations.outerChord),
		proofbound.ProductUpper(2, outerPointError))
	innerRadius = proofbound.AbsSumUpper(innerRadius,
		proofbound.ProductUpper(2, stations.innerChord),
		proofbound.ProductUpper(2, innerPointError))
	outerEdge = proofbound.AbsSumUpper(outerEdge, proofbound.ProductUpper(4, outerPointError))
	innerEdge = proofbound.AbsSumUpper(innerEdge, proofbound.ProductUpper(4, innerPointError))
	step := proofbound.RatFloatUp(new(big.Rat).Quo(
		proofbound.IntervalAbsUpper(thetaIv), big.NewRat(twistShellAxialCells, 1)))
	var held twistShellHeld
	outer, inner := make([][]int, twistShellAxialCells+1), make([][]int, twistShellAxialCells+1)
	add := func(v r3.Vec) int {
		held.verts = append(held.verts, v)
		return len(held.verts) - 1
	}
	for layer := 0; layer <= twistShellAxialCells; layer++ {
		if err := ctx.Err(); err != nil {
			return twistShellHeld{}, err
		}
		fraction := big.NewRat(int64(layer), twistShellAxialCells)
		outerZ := height * float64(layer) / twistShellAxialCells
		innerZ := thickness + (height-thickness)*float64(layer)/twistShellAxialCells
		outerExactZ := new(big.Rat).Mul(twistRat(height), fraction)
		innerExactZ := new(big.Rat).Add(twistRat(thickness),
			new(big.Rat).Mul(new(big.Rat).Sub(twistRat(height), twistRat(thickness)), fraction))
		if layer == twistShellAxialCells {
			outerZ, innerZ = height, height
		}
		outerAngleIv := proofbound.IntervalScale(thetaIv, fraction)
		innerFraction := new(big.Rat).Quo(innerExactZ, twistRat(height))
		innerAngleIv := proofbound.IntervalScale(thetaIv, innerFraction)
		outerTheta := theta * float64(layer) / twistShellAxialCells
		innerTheta := theta * innerZ / height
		if layer == twistShellAxialCells {
			outerTheta, innerTheta = theta, theta
		}
		outer[layer], inner[layer] = make([]int, n), make([]int, n)
		for i := range n {
			v, e, err := twistShellRingVertex(stations.outer[i], stations.outerVertex[i],
				outerZ, outerExactZ, outerAngleIv, outerTheta)
			if err != nil {
				return twistShellHeld{}, err
			}
			outer[layer][i] = add(v)
			held.vertexError = max(held.vertexError, e)
			v, e, err = twistShellRingVertex(stations.inner[i], stations.innerVertex[i],
				innerZ, innerExactZ, innerAngleIv, innerTheta)
			if err != nil {
				return twistShellHeld{}, err
			}
			inner[layer][i] = add(v)
			held.vertexError = max(held.vertexError, e)
		}
	}
	bottom := add(r3.NewVec(0, 0, 0))
	floor := add(r3.NewVec(0, 0, thickness))
	tri := func(a, b, c, group int) {
		held.tris = append(held.tris, [3]int{a, b, c})
		held.src = append(held.src, group)
	}
	const (
		outerWall = iota
		innerWall
		bottomCap
		floorCap
		topRim
	)
	for layer := range twistShellAxialCells {
		for i := range n {
			j := (i + 1) % n
			a, b, c, d := outer[layer][i], outer[layer][j], outer[layer+1][j], outer[layer+1][i]
			tri(a, b, c, outerWall)
			tri(a, c, d, outerWall)
			a, b, c, d = inner[layer][i], inner[layer][j], inner[layer+1][j], inner[layer+1][i]
			tri(a, c, b, innerWall)
			tri(a, d, c, innerWall)
		}
	}
	for i := range n {
		j := (i + 1) % n
		tri(bottom, outer[0][j], outer[0][i], bottomCap)
		tri(floor, inner[0][i], inner[0][j], floorCap)
		a, b, c, d := outer[twistShellAxialCells][i], outer[twistShellAxialCells][j],
			inner[twistShellAxialCells][j], inner[twistShellAxialCells][i]
		tri(a, b, c, topRim)
		tri(a, c, d, topRim)
	}
	departure := func(chord, radius, edge float64) float64 {
		interp := proofbound.ProductUpper(proofbound.ProductUpper(radius,
			proofbound.ProductUpper(step, step)), 0.125)
		warp := proofbound.ProductUpper(proofbound.ProductUpper(edge, step), 0.25)
		return proofbound.AbsSumUpper(chord, interp, warp, held.vertexError)
	}
	held.outerBound = departure(stations.outerChord, outerRadius, outerEdge)
	held.innerBound = departure(stations.innerChord, innerRadius, innerEdge)
	held.capBound = proofbound.AbsSumUpper(max(stations.outerChord, stations.innerChord), held.vertexError)
	// On a straight homotopy from each exact wall patch to its two held
	// triangles, the normal travel is bounded by departure. The two
	// parameter derivatives of the homotopy are bounded by the sums of
	// their exact and held lengths. Horizontal caps and the rim remain in
	// their own plane, so their homotopies sweep no occupied volume.
	var trueWallArea float64
	wallAllow := func(rings [][]int, spans []float64, delta, radius, heightSpan, angleSpan float64) {
		for layer := range twistShellAxialCells {
			trueD := proofbound.AbsSumUpper(heightSpan,
				proofbound.ProductUpper(radius, angleSpan))
			for i, trueL := range spans {
				j := (i + 1) % n
				a, b := held.verts[rings[layer][i]], held.verts[rings[layer][j]]
				c, d := held.verts[rings[layer+1][i]], held.verts[rings[layer+1][j]]
				heldL := max(twistShellDistanceUpper(a, b), twistShellDistanceUpper(c, d))
				heldD := max(twistShellDistanceUpper(a, c), twistShellDistanceUpper(b, d))
				area := proofbound.ProductUpper(trueL, trueD)
				trueWallArea = proofbound.AbsSumUpper(trueWallArea, area)
				held.volumeBound = proofbound.AbsSumUpper(held.volumeBound,
					proofbound.ProductUpper(delta, proofbound.ProductUpper(
						proofbound.AbsSumUpper(trueL, heldL), proofbound.AbsSumUpper(trueD, heldD))))
			}
		}
	}
	wallAllow(outer, stations.outerSpanLen, held.outerBound, outerRadius,
		proofbound.DivUpper(height, twistShellAxialCells), step)
	wallAllow(inner, stations.innerSpanLen, held.innerBound, innerRadius,
		proofbound.DivUpper(height-thickness, twistShellAxialCells),
		proofbound.ProductUpper(step, proofbound.DivUpper(height-thickness, height)))
	heldArea := proofbound.PerturbedAreaUpper(held.verts, held.tris, 0)
	// The top rim, outside bottom, and inside floor have combined exact area
	// 2*A(outer). Every wall is charged separately by its parameter cell.
	// This intentionally broad non-cancelling area term is safe for subsets.
	held.areaSlack = proofbound.AbsSumUpper(trueWallArea,
		proofbound.ProductUpper(2, proofbound.RatFloatUp(outerArea.Hi)), heldArea)
	if proofbound.IsNonFinite(held.outerBound) || proofbound.IsNonFinite(held.innerBound) ||
		proofbound.IsNonFinite(held.volumeBound) || proofbound.IsNonFinite(held.areaSlack) {
		return twistShellHeld{}, fmt.Errorf(`%w: twisted shell mesh has no finite proof`, ErrUnsupported)
	}
	if err := loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), held.verts, held.tris); err != nil {
		return twistShellHeld{}, fmt.Errorf("twisted shell held boundary: %w", err)
	}
	if _, err := facetproof.AuditFacetedMesh(ctx, held.verts, held.tris); err != nil {
		return twistShellHeld{}, fmt.Errorf("twisted shell held topology: %w", err)
	}
	return held, nil
}

func twistShellFacetedBody(ctx context.Context, d *Document, ref producerID,
	held twistShellHeld, outerArea, innerArea proofbound.RatInterval,
	height, thickness float64) (*Body, error) {
	if len(held.verts) == 0 || len(held.tris) == 0 || len(held.src) != len(held.tris) {
		return nil, fmt.Errorf(`%w: twisted shell has no complete held boundary`, ErrUnsupported)
	}
	trueVolume := proofbound.IntervalSub(
		proofbound.IntervalScale(outerArea, twistRat(height)),
		proofbound.IntervalScale(innerArea,
			new(big.Rat).Sub(twistRat(height), twistRat(thickness))))
	if trueVolume.Lo.Sign() <= 0 {
		return nil, fmt.Errorf(`%w: twisted shell has no positive volume proof`, ErrUnsupported)
	}
	audit, err := facetproof.AuditFacetedMesh(ctx, held.verts, held.tris)
	if err != nil {
		return nil, err
	}
	heldVolume := new(big.Rat)
	for _, volume := range audit.CompVol {
		heldVolume.Add(heldVolume, volume)
	}
	if heldVolume.Sign() <= 0 || proofbound.IntervalAbsUpper(proofbound.IntervalSub(
		trueVolume, proofbound.PointInterval(heldVolume))).Cmp(twistRat(held.volumeBound)) > 0 {
		return nil, fmt.Errorf(`%w: twisted shell occupied-volume allowance misses the source reading`, ErrUnsupported)
	}
	bound := max(held.outerBound, held.innerBound, held.capBound)
	vertexBound := make([]float64, len(held.verts))
	for i := range vertexBound {
		vertexBound[i] = bound
	}
	groups := []facetGroup{
		{origins: []FeatureRef{{producer: ref, Role: prismOuterFirstSideRole}}},
		{origins: []FeatureRef{{producer: ref, Role: "shellSide(0,0)"}}},
		{origins: []FeatureRef{{producer: ref, Role: roleCapStart}}, planar: true},
		{origins: []FeatureRef{{producer: ref, Role: roleShellCap}}, planar: true},
		{origins: []FeatureRef{{producer: ref, Role: "rim(0)"}}, planar: true},
	}
	payload := facetedPayload{
		verts: held.verts, vertexBound: vertexBound, tris: held.tris,
		src: held.src, groups: groups, meshBound: bound,
		volSymDiff: held.volumeBound, areaSlack: held.areaSlack,
		dPair: facetedSourceDiameterUpper(held.verts, bound), xform: r3.Identity(),
		contactAudited: true,
	}
	return buildFacetedBodyWithProof(ctx, d, ref, payload, audit, Measurement{}, nil)
}
