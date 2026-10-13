package twistsweep

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveangle"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

const twistShellAxialCells = 8

// TwistShellHeld is the sampled cup mesh with the volume, area and error
// bounds held for it.
type TwistShellHeld struct {
	Verts       []r3.Vec
	Tris        [][3]int
	Src         []int
	VertexError float64
	OuterBound  float64
	InnerBound  float64
	CapBound    float64
	VolumeBound float64
	AreaSlack   float64
}

func twistShellDistanceUpper(a, b r3.Vec) float64 {
	dx := new(big.Rat).Sub(TwistRat(a.X), TwistRat(b.X))
	dy := new(big.Rat).Sub(TwistRat(a.Y), TwistRat(b.Y))
	dz := new(big.Rat).Sub(TwistRat(a.Z), TwistRat(b.Z))
	squared := new(big.Rat).Add(new(big.Rat).Add(
		new(big.Rat).Mul(dx, dx), new(big.Rat).Mul(dy, dy)), new(big.Rat).Mul(dz, dz))
	return proofbound.RatSqrtUp(squared)
}

func twistShellContourBounds(points []planePoint) (float64, float64) {
	var radius, edge float64
	for i, p := range points {
		q := points[(i+1)%len(points)]
		radiusRat := new(big.Rat).Add(new(big.Rat).Abs(TwistRat(p.U)),
			new(big.Rat).Abs(TwistRat(p.V)))
		edgeRat := new(big.Rat).Add(
			new(big.Rat).Abs(new(big.Rat).Sub(TwistRat(q.U), TwistRat(p.U))),
			new(big.Rat).Abs(new(big.Rat).Sub(TwistRat(q.V), TwistRat(p.V))))
		radius = max(radius, proofbound.RatFloatUp(radiusRat))
		edge = max(edge, proofbound.RatFloatUp(edgeRat))
	}
	return radius, edge
}

func twistShellRingVertex(p planePoint, sourceError, z float64, exactZ *big.Rat,
	angle proofbound.RatInterval, heldAngle float64) (r3.Vec, float64, error) {
	sinIv, cosIv, ok := proofbound.RadSinCosSpan(angle)
	if !ok {
		return r3.Vec{}, 0, fmt.Errorf(`%w: twisted shell ring angle is not bounded`, decaderr.ErrUnsupported)
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

// TwistShellSampledMesh builds the sampled twisted cup mesh from stations and
// returns it with the bounds held for it.
func TwistShellSampledMesh(ctx context.Context, stations twistOffsetStations,
	height, thickness float64, angle units.Value,
	outerArea proofbound.RatInterval) (TwistShellHeld, error) {
	if err := ctx.Err(); err != nil {
		return TwistShellHeld{}, err
	}
	if !TwistOffsetConvexPolygon(stations.Outer) || !TwistOffsetConvexPolygon(stations.Inner) ||
		len(stations.Outer) != len(stations.Inner) || height <= thickness || thickness <= 0 {
		return TwistShellHeld{}, fmt.Errorf(`%w: twisted shell has no paired convex contours`, decaderr.ErrUnsupported)
	}
	thetaIv, ok := revolveangle.FromValue(angle).Enclosure()
	theta, err := angle.In(units.Radian)
	if !ok || err != nil {
		return TwistShellHeld{}, fmt.Errorf(`%w: twisted shell has no bounded turn`, decaderr.ErrUnsupported)
	}
	n := len(stations.Outer)
	outerRadius, outerEdge := twistShellContourBounds(stations.Outer)
	innerRadius, innerEdge := twistShellContourBounds(stations.Inner)
	var outerPointError, innerPointError float64
	for i := range stations.Outer {
		outerPointError = max(outerPointError, stations.OuterVertex[i])
		innerPointError = max(innerPointError, stations.InnerVertex[i])
	}
	outerRadius = proofbound.AbsSumUpper(outerRadius,
		proofbound.ProductUpper(2, stations.OuterChord),
		proofbound.ProductUpper(2, outerPointError))
	innerRadius = proofbound.AbsSumUpper(innerRadius,
		proofbound.ProductUpper(2, stations.InnerChord),
		proofbound.ProductUpper(2, innerPointError))
	outerEdge = proofbound.AbsSumUpper(outerEdge, proofbound.ProductUpper(4, outerPointError))
	innerEdge = proofbound.AbsSumUpper(innerEdge, proofbound.ProductUpper(4, innerPointError))
	step := proofbound.RatFloatUp(new(big.Rat).Quo(
		proofbound.IntervalAbsUpper(thetaIv), big.NewRat(twistShellAxialCells, 1)))
	var held TwistShellHeld
	outer, inner := make([][]int, twistShellAxialCells+1), make([][]int, twistShellAxialCells+1)
	add := func(v r3.Vec) int {
		held.Verts = append(held.Verts, v)
		return len(held.Verts) - 1
	}
	for layer := 0; layer <= twistShellAxialCells; layer++ {
		if err := ctx.Err(); err != nil {
			return TwistShellHeld{}, err
		}
		fraction := big.NewRat(int64(layer), twistShellAxialCells)
		outerZ := height * float64(layer) / twistShellAxialCells
		innerZ := thickness + (height-thickness)*float64(layer)/twistShellAxialCells
		outerExactZ := new(big.Rat).Mul(TwistRat(height), fraction)
		innerExactZ := new(big.Rat).Add(TwistRat(thickness),
			new(big.Rat).Mul(new(big.Rat).Sub(TwistRat(height), TwistRat(thickness)), fraction))
		if layer == twistShellAxialCells {
			outerZ, innerZ = height, height
		}
		outerAngleIv := proofbound.IntervalScale(thetaIv, fraction)
		innerFraction := new(big.Rat).Quo(innerExactZ, TwistRat(height))
		innerAngleIv := proofbound.IntervalScale(thetaIv, innerFraction)
		outerTheta := theta * float64(layer) / twistShellAxialCells
		innerTheta := theta * innerZ / height
		if layer == twistShellAxialCells {
			outerTheta, innerTheta = theta, theta
		}
		outer[layer], inner[layer] = make([]int, n), make([]int, n)
		for i := range n {
			v, e, err := twistShellRingVertex(stations.Outer[i], stations.OuterVertex[i],
				outerZ, outerExactZ, outerAngleIv, outerTheta)
			if err != nil {
				return TwistShellHeld{}, err
			}
			outer[layer][i] = add(v)
			held.VertexError = max(held.VertexError, e)
			v, e, err = twistShellRingVertex(stations.Inner[i], stations.InnerVertex[i],
				innerZ, innerExactZ, innerAngleIv, innerTheta)
			if err != nil {
				return TwistShellHeld{}, err
			}
			inner[layer][i] = add(v)
			held.VertexError = max(held.VertexError, e)
		}
	}
	bottom := add(r3.NewVec(0, 0, 0))
	floor := add(r3.NewVec(0, 0, thickness))
	tri := func(a, b, c, group int) {
		held.Tris = append(held.Tris, [3]int{a, b, c})
		held.Src = append(held.Src, group)
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
		return proofbound.AbsSumUpper(chord, interp, warp, held.VertexError)
	}
	held.OuterBound = departure(stations.OuterChord, outerRadius, outerEdge)
	held.InnerBound = departure(stations.InnerChord, innerRadius, innerEdge)
	held.CapBound = proofbound.AbsSumUpper(max(stations.OuterChord, stations.InnerChord), held.VertexError)
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
				a, b := held.Verts[rings[layer][i]], held.Verts[rings[layer][j]]
				c, d := held.Verts[rings[layer+1][i]], held.Verts[rings[layer+1][j]]
				heldL := max(twistShellDistanceUpper(a, b), twistShellDistanceUpper(c, d))
				heldD := max(twistShellDistanceUpper(a, c), twistShellDistanceUpper(b, d))
				area := proofbound.ProductUpper(trueL, trueD)
				trueWallArea = proofbound.AbsSumUpper(trueWallArea, area)
				held.VolumeBound = proofbound.AbsSumUpper(held.VolumeBound,
					proofbound.ProductUpper(delta, proofbound.ProductUpper(
						proofbound.AbsSumUpper(trueL, heldL), proofbound.AbsSumUpper(trueD, heldD))))
			}
		}
	}
	wallAllow(outer, stations.outerSpanLen, held.OuterBound, outerRadius,
		proofbound.DivUpper(height, twistShellAxialCells), step)
	wallAllow(inner, stations.innerSpanLen, held.InnerBound, innerRadius,
		proofbound.DivUpper(height-thickness, twistShellAxialCells),
		proofbound.ProductUpper(step, proofbound.DivUpper(height-thickness, height)))
	heldArea := proofbound.PerturbedAreaUpper(held.Verts, held.Tris, 0)
	// The top rim, outside bottom, and inside floor have combined exact area
	// 2*A(outer). Every wall is charged separately by its parameter cell.
	// This intentionally broad non-cancelling area term is safe for subsets.
	held.AreaSlack = proofbound.AbsSumUpper(trueWallArea,
		proofbound.ProductUpper(2, proofbound.RatFloatUp(outerArea.Hi)), heldArea)
	if proofbound.IsNonFinite(held.OuterBound) || proofbound.IsNonFinite(held.InnerBound) ||
		proofbound.IsNonFinite(held.VolumeBound) || proofbound.IsNonFinite(held.AreaSlack) {
		return TwistShellHeld{}, fmt.Errorf(`%w: twisted shell mesh has no finite proof`, decaderr.ErrUnsupported)
	}
	if err := loftmesh.LoftCrossingAudit(proofbound.NewWorkBudget(ctx), held.Verts, held.Tris); err != nil {
		return TwistShellHeld{}, fmt.Errorf("twisted shell held boundary: %w", err)
	}
	if _, err := facetproof.AuditFacetedMesh(ctx, held.Verts, held.Tris); err != nil {
		return TwistShellHeld{}, fmt.Errorf("twisted shell held topology: %w", err)
	}
	return held, nil
}
