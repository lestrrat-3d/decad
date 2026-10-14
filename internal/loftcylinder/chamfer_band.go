package loftcylinder

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// ChamferEdge returns the undirected key of edge ab.
func ChamferEdge(a, b int) [2]int {
	if a > b {
		a, b = b, a
	}
	return [2]int{a, b}
}

func chamferRingEdges(ring []int) map[[2]int]struct{} {
	edges := make(map[[2]int]struct{}, len(ring))
	for i, a := range ring {
		edges[ChamferEdge(a, ring[(i+1)%len(ring)])] = struct{}{}
	}
	return edges
}

// ChamferBandBoundary returns the side-section edges of one cap band.
// The band is one triangle-connected annulus. Its only free edges are the
// recorded side section and inset; the full shell's vertex-link audit then
// rules out a pinch at any of their shared vertices.
func ChamferBandBoundary(tris [][3]int, band [2]int,
	side, inset []int) (map[[2]int]struct{}, error) {
	if band[0] < 0 || band[1] > len(tris) || band[0] >= band[1] ||
		len(side) < 3 || len(inset) < 3 {
		return nil, fmt.Errorf("%w: loft/cylinder band is incomplete", decaderr.ErrUnsupported)
	}
	sideEdges := chamferRingEdges(side)
	insetEdges := chamferRingEdges(inset)
	if len(sideEdges) != len(side) || len(insetEdges) != len(inset) {
		return nil, fmt.Errorf("%w: loft/cylinder band repeats a rim edge", decaderr.ErrUnsupported)
	}
	uses := make(map[[2]int][]int, 3*(band[1]-band[0]))
	for i := band[0]; i < band[1]; i++ {
		tri := tris[i]
		for k := range 3 {
			key := ChamferEdge(tri[k], tri[(k+1)%3])
			uses[key] = append(uses[key], i)
		}
	}
	neighbors := make([][]int, band[1]-band[0])
	boundary := make(map[[2]int]struct{}, len(sideEdges)+len(insetEdges))
	for edge, faces := range uses {
		switch len(faces) {
		case 1:
			if _, side := sideEdges[edge]; !side {
				if _, inset := insetEdges[edge]; !inset {
					return nil, fmt.Errorf("%w: loft/cylinder band has an unrecorded boundary", decaderr.ErrUnsupported)
				}
			}
			boundary[edge] = struct{}{}
		case 2:
			if _, side := sideEdges[edge]; side {
				return nil, fmt.Errorf("%w: loft/cylinder side rim is internal to a band", decaderr.ErrUnsupported)
			}
			if _, inset := insetEdges[edge]; inset {
				return nil, fmt.Errorf("%w: loft/cylinder inset rim is internal to a band", decaderr.ErrUnsupported)
			}
			a, b := faces[0]-band[0], faces[1]-band[0]
			neighbors[a] = append(neighbors[a], b)
			neighbors[b] = append(neighbors[b], a)
		default:
			return nil, fmt.Errorf("%w: loft/cylinder band has a nonmanifold edge", decaderr.ErrUnsupported)
		}
	}
	if len(boundary) != len(sideEdges)+len(insetEdges) {
		return nil, fmt.Errorf("%w: loft/cylinder band omits a rim edge", decaderr.ErrUnsupported)
	}
	seen := make([]bool, len(neighbors))
	seen[0] = true
	queue := []int{0}
	for head := 0; head < len(queue); head++ {
		for _, next := range neighbors[queue[head]] {
			if !seen[next] {
				seen[next] = true
				queue = append(queue, next)
			}
		}
	}
	if len(queue) != len(seen) {
		return nil, fmt.Errorf("%w: loft/cylinder band has disconnected interior patches", decaderr.ErrUnsupported)
	}
	return sideEdges, nil
}

// ChamferIdealBandLevels checks the axial levels of an ideal cap band.
// Every open band facet lies strictly between the source cap and its side
// section. Its only cap-plane edges belong to the inset loop, so the old cap
// disk/annulus can meet the ideal cutter only on that recorded inset rim.
func ChamferIdealBandLevels(tris [][3]int,
	xverts []proofarith.Xpt, band [2]int, inset []int, capLevel, sideLevel *big.Rat) error {
	if capLevel == nil || sideLevel == nil || capLevel.Cmp(sideLevel) == 0 {
		return fmt.Errorf("%w: ideal cap band has no axial slab", decaderr.ErrUnsupported)
	}
	insetEdges := chamferRingEdges(inset)
	for i := band[0]; i < band[1]; i++ {
		tri := tris[i]
		capCount, sideCount := 0, 0
		for k, vertex := range tri {
			z := meshbool.RatCoordOf(xverts[vertex], 2)
			switch {
			case z.Cmp(capLevel) == 0:
				capCount++
			case z.Cmp(sideLevel) == 0:
				sideCount++
			default:
				return fmt.Errorf("%w: ideal cap band leaves its two axial levels", decaderr.ErrUnsupported)
			}
			if z.Cmp(capLevel) == 0 &&
				meshbool.RatCoordOf(xverts[tri[(k+1)%3]], 2).Cmp(capLevel) == 0 {
				if _, ok := insetEdges[ChamferEdge(vertex, tri[(k+1)%3])]; !ok {
					return fmt.Errorf("%w: ideal cap band meets the cap away from its inset", decaderr.ErrUnsupported)
				}
			}
		}
		if capCount == 0 || sideCount == 0 {
			return fmt.Errorf("%w: ideal cap band has a facet outside its open slab", decaderr.ErrUnsupported)
		}
	}
	return nil
}

func chamferExactXY(p proofarith.Xpt) XY {
	return XY{X: meshbool.RatCoordOf(p, 0), Y: meshbool.RatCoordOf(p, 1)}
}

// ChamferExactRingSimple reports whether the exact ring is simple and
// counterclockwise around the origin.
// Every decision uses the rational side-section vertex, not its rounded
// stored image. The origin witness is the one certified for the source ring.
func ChamferExactRingSimple(ctx context.Context, xverts []proofarith.Xpt, ring []int) bool {
	n := len(ring)
	if n < 3 {
		return false
	}
	points := make([]XY, n)
	filtered := make([]meshbool.Xp2, n)
	for i, index := range ring {
		if index < 0 || index >= len(xverts) {
			return false
		}
		points[i] = chamferExactXY(xverts[index])
		filtered[i] = meshbool.NewXP2(points[i].X, points[i].Y)
	}
	area2 := new(big.Rat)
	inside := false
	for i := range points {
		if i%32 == 0 && ctx.Err() != nil {
			return false
		}
		a, b := points[i], points[(i+1)%n]
		if a.X.Cmp(b.X) == 0 && a.Y.Cmp(b.Y) == 0 {
			return false
		}
		area2.Add(area2, proofarith.SubRat(new(big.Rat),
			proofarith.MulRat(new(big.Rat), a.X, b.Y), proofarith.MulRat(new(big.Rat), a.Y, b.X)))
		if (a.Y.Sign() > 0) != (b.Y.Sign() > 0) {
			x := proofarith.SubRat(new(big.Rat), a.X, new(big.Rat).Quo(proofarith.MulRat(new(big.Rat), a.Y,
				proofarith.SubRat(new(big.Rat), b.X, a.X)), proofarith.SubRat(new(big.Rat), b.Y, a.Y)))
			if x.Sign() == 0 {
				return false
			}
			if x.Sign() > 0 {
				inside = !inside
			}
		}
		for j := i + 1; j < n; j++ {
			if j == i+1 || i == 0 && j == n-1 {
				continue
			}
			if edgesMeetXP(filtered[i], filtered[(i+1)%n],
				filtered[j], filtered[(j+1)%n]) {
				return false
			}
		}
	}
	return area2.Sign() > 0 && inside
}

// ChamferExactNested proves the exact inner ring lies strictly inside the
// exact outer ring.
func ChamferExactNested(ctx context.Context, xverts []proofarith.Xpt,
	outer, inner []int) (bool, error) {
	if len(outer) < 3 || len(inner) < 3 {
		return false, nil
	}
	outerFiltered := make([]meshbool.Xp2, len(outer))
	for i, index := range outer {
		if i%64 == 0 {
			if err := ctx.Err(); err != nil {
				return false, err
			}
		}
		p := chamferExactXY(xverts[index])
		outerFiltered[i] = meshbool.NewXP2(p.X, p.Y)
	}
	innerFiltered := make([]meshbool.Xp2, len(inner))
	for i, index := range inner {
		if i%64 == 0 {
			if err := ctx.Err(); err != nil {
				return false, err
			}
		}
		p := chamferExactXY(xverts[index])
		innerFiltered[i] = meshbool.NewXP2(p.X, p.Y)
	}
	for i := range outerFiltered {
		for j := range innerFiltered {
			if j%64 == 0 {
				if err := ctx.Err(); err != nil {
					return false, err
				}
			}
			if edgesMeetXP(outerFiltered[i], outerFiltered[(i+1)%len(outerFiltered)],
				innerFiltered[j], innerFiltered[(j+1)%len(innerFiltered)]) {
				return false, nil
			}
		}
	}
	inside, on, err := meshbool.PointInPoly2(proofbound.NewWorkBudget(ctx), outerFiltered, innerFiltered[0])
	return inside && !on, err
}

// ChamferContactOnSide reports whether p lies on a recorded side edge of tri.
func ChamferContactOnSide(p proofarith.Xpt, tri [3]int,
	xverts []proofarith.Xpt, sideEdges map[[2]int]struct{}) bool {
	for k := range 3 {
		a, b := tri[k], tri[(k+1)%3]
		if _, ok := sideEdges[ChamferEdge(a, b)]; !ok {
			continue
		}
		line := proofarith.Xsub(xverts[b], xverts[a])
		toPoint := proofarith.Xsub(p, xverts[a])
		cross := proofarith.Xcross(line, toPoint)
		if cross.X.Sign() == 0 && cross.Y.Sign() == 0 && cross.Z.Sign() == 0 &&
			meshbool.PointOnSegment3D(xverts[a], xverts[b], p) {
			return true
		}
	}
	return false
}

// ChamferIdealBandAudit checks the exact geometry of the two cap bands.
// The only facets not inherited from the old embedded shell or an audited
// planar inset cap are the two bands. Check their exact rational geometry,
// including every pair that could meet after the side section is rounded.
func ChamferIdealBandAudit(ctx context.Context, vertexCount int, tris [][3]int,
	xverts []proofarith.Xpt, bands [2][2]int, clipBound float64) error {
	if len(xverts) != vertexCount || proofbound.IsNonFinite(clipBound) || clipBound < 0 {
		return fmt.Errorf("%w: ideal cap band has no finite station proof", decaderr.ErrUnsupported)
	}
	approx := make([]r3.Vec, len(xverts))
	maxRound := 0.0
	for i, p := range xverts {
		approx[i] = p.Vec()
		if !proofbound.FiniteVec(approx[i]) {
			return fmt.Errorf("%w: ideal cap band has a nonfinite station", decaderr.ErrUnsupported)
		}
		maxRound = max(maxRound,
			proofarith.RationalFloatError(meshbool.RatCoordOf(p, 0), approx[i].X),
			proofarith.RationalFloatError(meshbool.RatCoordOf(p, 1), approx[i].Y),
			proofarith.RationalFloatError(meshbool.RatCoordOf(p, 2), approx[i].Z))
	}
	if proofbound.Radius3D(maxRound) > clipBound {
		return fmt.Errorf("%w: ideal cap band exceeds its side-station rounding bound", decaderr.ErrUnsupported)
	}
	pad := math.Nextafter(maxRound, math.Inf(1))
	const maxPairs = 32_000_000
	budget := proofbound.NewWorkBudget(ctx)
	for cap, band := range bands {
		if band[0] == band[1] {
			continue
		}
		tris := tris[band[0]:band[1]]
		pairCount := uint64(len(tris)) * uint64(len(tris)-1) / 2
		if pairCount > maxPairs {
			return fmt.Errorf("%w: ideal cap band exceeds its exact pair ceiling", decaderr.ErrUnsupported)
		}
		data := loftmesh.NewLoftAuditDataWithExact(approx, xverts, tris)
		boxes := make([][2]r3.Vec, len(tris))
		for i, tri := range tris {
			n := data.Norms[i]
			if n.X.Sign() == 0 && n.Y.Sign() == 0 && n.Z.Sign() == 0 {
				return fmt.Errorf("%w: ideal cap band has a collapsed triangle", decaderr.ErrUnsupported)
			}
			box := meshbool.TriBox(approx, tri)
			box[0] = r3.Vec{
				X: math.Nextafter(box[0].X-pad, math.Inf(-1)),
				Y: math.Nextafter(box[0].Y-pad, math.Inf(-1)),
				Z: math.Nextafter(box[0].Z-pad, math.Inf(-1)),
			}
			box[1] = r3.Vec{
				X: math.Nextafter(box[1].X+pad, math.Inf(1)),
				Y: math.Nextafter(box[1].Y+pad, math.Inf(1)),
				Z: math.Nextafter(box[1].Z+pad, math.Inf(1)),
			}
			boxes[i] = box
		}
		for i := range tris {
			for j := i + 1; j < len(tris); j++ {
				if err := budget.Step(); err != nil {
					return err
				}
				if !meshbool.BoxesOverlap(boxes[i], boxes[j]) {
					continue
				}
				if _, err := loftmesh.AuditLoftPairData(data, tris, i, j,
					loftmesh.LoftAuditShortcuts{ExactInput: true}); err != nil {
					return fmt.Errorf("%w: ideal cap band %d facets %d,%d make contact: %v",
						decaderr.ErrUnsupported, cap, i, j, err)
				}
			}
		}
	}
	return budget.Err()
}

// ChamferIdealOrientation requires nonzero exact facets and a positive exact
// signed volume.
func ChamferIdealOrientation(ctx context.Context, tris [][3]int,
	xverts []proofarith.Xpt) error {
	volume6 := new(big.Rat)
	for i, tri := range tris {
		if i%128 == 0 {
			if err := ctx.Err(); err != nil {
				return err
			}
		}
		a, b, c := xverts[tri[0]], xverts[tri[1]], xverts[tri[2]]
		n := proofarith.Xcross(proofarith.Xsub(b, a), proofarith.Xsub(c, a))
		if n.X.Sign() == 0 && n.Y.Sign() == 0 && n.Z.Sign() == 0 {
			return fmt.Errorf("%w: ideal cap shell has a collapsed triangle", decaderr.ErrUnsupported)
		}
		proofarith.AddRat(volume6, volume6, proofarith.XdotRat(a, proofarith.Xcross(b, c)))
	}
	if volume6.Sign() <= 0 {
		return fmt.Errorf("%w: ideal cap shell has no outward orientation", decaderr.ErrUnsupported)
	}
	return nil
}

// ChamferLerp places the exact point of segment ab at an axial level and its
// rounded held image.
func ChamferLerp(a, b r3.Vec, level *big.Rat, heldLevel float64) (r3.Vec, proofarith.Xpt, float64, error) {
	den := proofarith.SubRat(new(big.Rat), proofarith.FloatRat(b.Z), proofarith.FloatRat(a.Z))
	if den.Sign() == 0 {
		return r3.Vec{}, proofarith.Xpt{}, 0,
			fmt.Errorf("%w: loft/cylinder side has zero axial span", decaderr.ErrUnsupported)
	}
	t := new(big.Rat).Quo(proofarith.SubRat(new(big.Rat), level, proofarith.FloatRat(a.Z)), den)
	if t.Sign() <= 0 || t.Cmp(big.NewRat(1, 1)) >= 0 {
		return r3.Vec{}, proofarith.Xpt{}, 0,
			fmt.Errorf("%w: chamfer side level lies outside a wall", decaderr.ErrUnsupported)
	}
	exact := proofarith.Xlerp(proofarith.XptOf(a), proofarith.XptOf(b), new(big.Int).Set(t.Num()), new(big.Int).Set(t.Denom()))
	p := exact.Vec()
	p.Z = heldLevel
	if !proofbound.FiniteVec(p) {
		return r3.Vec{}, proofarith.Xpt{}, 0,
			fmt.Errorf("%w: chamfer side station is nonfinite", decaderr.ErrUnsupported)
	}
	round := max(
		proofarith.RationalFloatError(meshbool.RatCoordOf(exact, 0), p.X),
		proofarith.RationalFloatError(meshbool.RatCoordOf(exact, 1), p.Y),
		proofarith.RationalFloatError(meshbool.RatCoordOf(exact, 2), p.Z))
	return p, exact, proofbound.Radius3D(round), nil
}

// ChamferRingNested reports whether the inner ring lies inside the outer ring.
// Both rings are simple and counterclockwise on entry. Exact segment tests
// plus one strict inside witness prove the whole inner ring is nested.
func ChamferRingNested(ctx context.Context, verts []r3.Vec, outer, inner []int) (bool, error) {
	if len(outer) < 3 || len(inner) < 3 {
		return false, nil
	}
	for i, a := range outer {
		b := outer[(i+1)%len(outer)]
		pa, pb := verts[a], verts[b]
		for j, c := range inner {
			if j%64 == 0 {
				if err := ctx.Err(); err != nil {
					return false, err
				}
			}
			d := inner[(j+1)%len(inner)]
			pc, pd := verts[c], verts[d]
			if max(pa.X, pb.X) < min(pc.X, pd.X) || max(pc.X, pd.X) < min(pa.X, pb.X) ||
				max(pa.Y, pb.Y) < min(pc.Y, pd.Y) || max(pc.Y, pd.Y) < min(pa.Y, pb.Y) {
				continue
			}
			if EdgesMeet(ExactXY(pa), ExactXY(pb),
				ExactXY(pc), ExactXY(pd)) {
				return false, fmt.Errorf("%w: cap rings meet at outer edge %d (%v,%v) and inner edge %d (%v,%v)",
					decaderr.ErrUnsupported, i, pa, pb, j, pc, pd)
			}
		}
	}
	poly := make([]meshbool.Xp2, len(outer))
	for i, index := range outer {
		p := verts[index]
		poly[i] = meshbool.NewXP2(proofarith.FloatRat(p.X), proofarith.FloatRat(p.Y))
	}
	p := verts[inner[0]]
	inside, on, err := meshbool.PointInPoly2(proofbound.NewWorkBudget(ctx), poly,
		meshbool.NewXP2(proofarith.FloatRat(p.X), proofarith.FloatRat(p.Y)))
	return inside && !on, err
}
