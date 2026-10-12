package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

func loftCylinderChamferEdge(a, b int) [2]int {
	if a > b {
		a, b = b, a
	}
	return [2]int{a, b}
}

func loftCylinderChamferRingEdges(ring []int) map[[2]int]struct{} {
	edges := make(map[[2]int]struct{}, len(ring))
	for i, a := range ring {
		edges[loftCylinderChamferEdge(a, ring[(i+1)%len(ring)])] = struct{}{}
	}
	return edges
}

// The band is one triangle-connected annulus. Its only free edges are the
// recorded side section and inset; the full shell's vertex-link audit then
// rules out a pinch at any of their shared vertices.
func loftCylinderChamferBandBoundary(mesh *joinedMesh, band [2]int,
	side, inset []int) (map[[2]int]struct{}, error) {
	if band[0] < 0 || band[1] > len(mesh.tris) || band[0] >= band[1] ||
		len(side) < 3 || len(inset) < 3 {
		return nil, fmt.Errorf("%w: loft/cylinder band is incomplete", ErrUnsupported)
	}
	sideEdges := loftCylinderChamferRingEdges(side)
	insetEdges := loftCylinderChamferRingEdges(inset)
	if len(sideEdges) != len(side) || len(insetEdges) != len(inset) {
		return nil, fmt.Errorf("%w: loft/cylinder band repeats a rim edge", ErrUnsupported)
	}
	uses := make(map[[2]int][]int, 3*(band[1]-band[0]))
	for i := band[0]; i < band[1]; i++ {
		tri := mesh.tris[i]
		for k := range 3 {
			key := loftCylinderChamferEdge(tri[k], tri[(k+1)%3])
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
					return nil, fmt.Errorf("%w: loft/cylinder band has an unrecorded boundary", ErrUnsupported)
				}
			}
			boundary[edge] = struct{}{}
		case 2:
			if _, side := sideEdges[edge]; side {
				return nil, fmt.Errorf("%w: loft/cylinder side rim is internal to a band", ErrUnsupported)
			}
			if _, inset := insetEdges[edge]; inset {
				return nil, fmt.Errorf("%w: loft/cylinder inset rim is internal to a band", ErrUnsupported)
			}
			a, b := faces[0]-band[0], faces[1]-band[0]
			neighbors[a] = append(neighbors[a], b)
			neighbors[b] = append(neighbors[b], a)
		default:
			return nil, fmt.Errorf("%w: loft/cylinder band has a nonmanifold edge", ErrUnsupported)
		}
	}
	if len(boundary) != len(sideEdges)+len(insetEdges) {
		return nil, fmt.Errorf("%w: loft/cylinder band omits a rim edge", ErrUnsupported)
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
		return nil, fmt.Errorf("%w: loft/cylinder band has disconnected interior patches", ErrUnsupported)
	}
	return sideEdges, nil
}

// Every open band facet lies strictly between the source cap and its side
// section. Its only cap-plane edges belong to the inset loop, so the old cap
// disk/annulus can meet the ideal cutter only on that recorded inset rim.
func loftCylinderChamferIdealBandLevels(mesh *joinedMesh,
	xverts []proofarith.Xpt, band [2]int, inset []int, capLevel, sideLevel *big.Rat) error {
	if capLevel == nil || sideLevel == nil || capLevel.Cmp(sideLevel) == 0 {
		return fmt.Errorf("%w: ideal cap band has no axial slab", ErrUnsupported)
	}
	insetEdges := loftCylinderChamferRingEdges(inset)
	for i := band[0]; i < band[1]; i++ {
		tri := mesh.tris[i]
		capCount, sideCount := 0, 0
		for k, vertex := range tri {
			z := meshbool.RatCoordOf(xverts[vertex], 2)
			switch {
			case z.Cmp(capLevel) == 0:
				capCount++
			case z.Cmp(sideLevel) == 0:
				sideCount++
			default:
				return fmt.Errorf("%w: ideal cap band leaves its two axial levels", ErrUnsupported)
			}
			if z.Cmp(capLevel) == 0 &&
				meshbool.RatCoordOf(xverts[tri[(k+1)%3]], 2).Cmp(capLevel) == 0 {
				if _, ok := insetEdges[loftCylinderChamferEdge(vertex, tri[(k+1)%3])]; !ok {
					return fmt.Errorf("%w: ideal cap band meets the cap away from its inset", ErrUnsupported)
				}
			}
		}
		if capCount == 0 || sideCount == 0 {
			return fmt.Errorf("%w: ideal cap band has a facet outside its open slab", ErrUnsupported)
		}
	}
	return nil
}

func loftCylinderChamferExactXY(p proofarith.Xpt) loftCylinderXY {
	return loftCylinderXY{x: meshbool.RatCoordOf(p, 0), y: meshbool.RatCoordOf(p, 1)}
}

// Every decision uses the rational side-section vertex, not its rounded
// stored image. The origin witness is the one certified for the source ring.
func loftCylinderChamferExactRingSimple(ctx context.Context, xverts []proofarith.Xpt, ring []int) bool {
	n := len(ring)
	if n < 3 {
		return false
	}
	points := make([]loftCylinderXY, n)
	for i, index := range ring {
		if index < 0 || index >= len(xverts) {
			return false
		}
		points[i] = loftCylinderChamferExactXY(xverts[index])
	}
	area2 := new(big.Rat)
	inside := false
	for i := range points {
		if i%32 == 0 && ctx.Err() != nil {
			return false
		}
		a, b := points[i], points[(i+1)%n]
		if a.x.Cmp(b.x) == 0 && a.y.Cmp(b.y) == 0 {
			return false
		}
		area2.Add(area2, new(big.Rat).Sub(
			new(big.Rat).Mul(a.x, b.y), new(big.Rat).Mul(a.y, b.x)))
		if (a.y.Sign() > 0) != (b.y.Sign() > 0) {
			x := new(big.Rat).Sub(a.x, new(big.Rat).Quo(new(big.Rat).Mul(a.y,
				new(big.Rat).Sub(b.x, a.x)), new(big.Rat).Sub(b.y, a.y)))
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
			if loftCylinderEdgesMeet(a, b, points[j], points[(j+1)%n]) {
				return false
			}
		}
	}
	return area2.Sign() > 0 && inside
}

func loftCylinderChamferExactNested(ctx context.Context, xverts []proofarith.Xpt,
	outer, inner []int) (bool, error) {
	if len(outer) < 3 || len(inner) < 3 {
		return false, nil
	}
	for i, a := range outer {
		b := outer[(i+1)%len(outer)]
		pa, pb := loftCylinderChamferExactXY(xverts[a]), loftCylinderChamferExactXY(xverts[b])
		for j, c := range inner {
			if j%64 == 0 {
				if err := ctx.Err(); err != nil {
					return false, err
				}
			}
			d := inner[(j+1)%len(inner)]
			pc, pd := loftCylinderChamferExactXY(xverts[c]), loftCylinderChamferExactXY(xverts[d])
			if loftCylinderEdgesMeet(pa, pb, pc, pd) {
				return false, nil
			}
		}
	}
	poly := make([]meshbool.Xp2, len(outer))
	for i, index := range outer {
		p := loftCylinderChamferExactXY(xverts[index])
		poly[i] = meshbool.NewXP2(p.x, p.y)
	}
	p := loftCylinderChamferExactXY(xverts[inner[0]])
	inside, on, err := meshbool.PointInPoly2(proofbound.NewWorkBudget(ctx), poly, meshbool.NewXP2(p.x, p.y))
	return inside && !on, err
}

func loftCylinderChamferContactOnSide(p proofarith.Xpt, tri [3]int,
	xverts []proofarith.Xpt, sideEdges map[[2]int]struct{}) bool {
	for k := range 3 {
		a, b := tri[k], tri[(k+1)%3]
		if _, ok := sideEdges[loftCylinderChamferEdge(a, b)]; !ok {
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

// The ideal side stations are exact rational points on the old wall. Their
// rounded output images are charged separately after this ideal audit.
func loftCylinderChamferIdealVertices(verts []r3.Vec, side loftCylinderChamferSide) []proofarith.Xpt {
	xverts := make([]proofarith.Xpt, len(verts))
	for i, v := range verts {
		xverts[i] = proofarith.XptOf(v)
	}
	for cap, ring := range side.rings {
		for i, vertex := range ring {
			xverts[vertex] = side.exact[cap][i]
		}
	}
	return xverts
}

// The only facets not inherited from the old embedded shell or an audited
// planar inset cap are the two bands. Check their exact rational geometry,
// including every pair that could meet after the side section is rounded.
func loftCylinderChamferIdealBandAudit(ctx context.Context, mesh *joinedMesh,
	xverts []proofarith.Xpt, bands [2][2]int, clipBound float64) error {
	if len(xverts) != len(mesh.verts) || proofbound.IsNonFinite(clipBound) || clipBound < 0 {
		return fmt.Errorf("%w: ideal cap band has no finite station proof", ErrUnsupported)
	}
	approx := make([]r3.Vec, len(xverts))
	maxRound := 0.0
	for i, p := range xverts {
		approx[i] = p.Vec()
		if !proofbound.FiniteVec(approx[i]) {
			return fmt.Errorf("%w: ideal cap band has a nonfinite station", ErrUnsupported)
		}
		maxRound = max(maxRound,
			proofarith.RationalFloatError(meshbool.RatCoordOf(p, 0), approx[i].X),
			proofarith.RationalFloatError(meshbool.RatCoordOf(p, 1), approx[i].Y),
			proofarith.RationalFloatError(meshbool.RatCoordOf(p, 2), approx[i].Z))
	}
	if proofbound.Radius3D(maxRound) > clipBound {
		return fmt.Errorf("%w: ideal cap band exceeds its side-station rounding bound", ErrUnsupported)
	}
	pad := math.Nextafter(maxRound, math.Inf(1))
	const maxPairs = 32_000_000
	budget := proofbound.NewWorkBudget(ctx)
	for cap, band := range bands {
		if band[0] == band[1] {
			continue
		}
		tris := mesh.tris[band[0]:band[1]]
		pairCount := uint64(len(tris)) * uint64(len(tris)-1) / 2
		if pairCount > maxPairs {
			return fmt.Errorf("%w: ideal cap band exceeds its exact pair ceiling", ErrUnsupported)
		}
		data := loftmesh.NewLoftAuditDataWithExact(approx, xverts, tris)
		boxes := make([][2]r3.Vec, len(tris))
		for i, tri := range tris {
			n := data.Norms[i]
			if n.X.Sign() == 0 && n.Y.Sign() == 0 && n.Z.Sign() == 0 {
				return fmt.Errorf("%w: ideal cap band has a collapsed triangle", ErrUnsupported)
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
						ErrUnsupported, cap, i, j, err)
				}
			}
		}
	}
	return budget.Err()
}

func loftCylinderChamferIdealOrientation(ctx context.Context, mesh *joinedMesh,
	xverts []proofarith.Xpt) error {
	volume6 := new(big.Rat)
	for i, tri := range mesh.tris {
		if i%128 == 0 {
			if err := ctx.Err(); err != nil {
				return err
			}
		}
		a, b, c := xverts[tri[0]], xverts[tri[1]], xverts[tri[2]]
		n := proofarith.Xcross(proofarith.Xsub(b, a), proofarith.Xsub(c, a))
		if n.X.Sign() == 0 && n.Y.Sign() == 0 && n.Z.Sign() == 0 {
			return fmt.Errorf("%w: ideal cap shell has a collapsed triangle", ErrUnsupported)
		}
		volume6.Add(volume6, proofarith.XdotRat(a, proofarith.Xcross(b, c)))
	}
	if volume6.Sign() <= 0 {
		return fmt.Errorf("%w: ideal cap shell has no outward orientation", ErrUnsupported)
	}
	return nil
}

// A mid-height point well inside the exact blank, and outside the bore when
// present, fixes the occupied side. The ideal middle side is an exact subset
// of the old wall's two-triangle strips, so it has the same classification
// there; the cap bands cannot reach this level.
func loftCylinderChamferSolidWitness(ctx context.Context, fp facetedPayload,
	source loftCylinderChamferSource, setback float64) error {
	mid := new(big.Rat).Add(proofarith.FloatRat(source.z0), proofarith.FloatRat(source.z1))
	mid.Quo(mid, big.NewRat(2, 1))
	start := new(big.Rat).Add(proofarith.FloatRat(source.z0), proofarith.FloatRat(setback))
	end := new(big.Rat).Sub(proofarith.FloatRat(source.z1), proofarith.FloatRat(setback))
	if mid.Cmp(start) <= 0 || mid.Cmp(end) >= 0 {
		return fmt.Errorf("%w: cap bands leave no common interior slice", ErrUnsupported)
	}
	r := 0.0
	if source.boreRadius > 0 {
		r = proofbound.RatFloatUp(new(big.Rat).Quo(new(big.Rat).Add(
			proofarith.FloatRat(source.boreRadius), proofarith.FloatRat(source.blankRadius)),
			big.NewRat(2, 1)))
	}
	if proofbound.AbsSumUpper(r, source.innerInset) >= source.blankRadius ||
		source.boreRadius > 0 && r <= proofbound.AbsSumUpper(source.boreRadius, fp.meshBound) {
		return fmt.Errorf("%w: cap chamfer lacks an interior blank witness", ErrUnsupported)
	}
	witness := proofarith.Xlerp(
		proofarith.XptOf(r3.Vec{X: r, Z: source.z0}),
		proofarith.XptOf(r3.Vec{X: r, Z: source.z1}),
		big.NewInt(1), big.NewInt(2))
	indices := make([]int, len(fp.tris))
	for i := range indices {
		indices[i] = i
	}
	inside, on, err := meshbool.MeshParityPreparedContext(ctx, witness,
		meshbool.NewParityMesh(fp.verts, fp.tris), indices)
	if err != nil {
		return err
	}
	if !inside || on {
		return fmt.Errorf("%w: cap chamfer shell excludes its blank witness", ErrUnsupported)
	}
	return nil
}

type loftCylinderChamferToothFace struct {
	body  *Body
	loft  loftPayload
	root  int
	cell  int
	bound float64
	scale *big.Rat
}

// Union copied these precise Loft face triangles, and Cut retained their
// groups. Match by authenticated origin instead of interpreting a role string
// on a faceted result that may coalesce several public edges.
func loftCylinderChamferToothFaces(source loftCylinderChamferSource) (map[FeatureRef][]loftCylinderChamferToothFace, error) {
	if source.blank == nil || len(source.teeth) == 0 ||
		len(source.originalTeeth) != len(source.teeth) || source.blankRadius <= 0 {
		return nil, fmt.Errorf("%w: loft/cylinder chamfer lost its source operands", ErrUnsupported)
	}
	faces := make(map[FeatureRef][]loftCylinderChamferToothFace)
	for toothIndex, tooth := range source.teeth {
		if tooth == nil || source.originalTeeth[toothIndex] == nil {
			return nil, fmt.Errorf("%w: loft/cylinder tooth source is absent", ErrUnsupported)
		}
		lp, ok := tooth.payload.(loftPayload)
		original, originalOK := source.originalTeeth[toothIndex].payload.(loftPayload)
		if !ok || !lp.authenticatedReconstruction || lp.sourceIdentity == nil ||
			!originalOK || original.sourceIdentity != lp.sourceIdentity ||
			lp.proof.FacetDeparture < 0 || proofbound.IsNonFinite(lp.proof.FacetDeparture) ||
			len(lp.segment) != 1 {
			return nil, fmt.Errorf("%w: loft/cylinder tooth lost its certified surface", ErrUnsupported)
		}
		root := -1
		for i, segment := range lp.segment[0] {
			if segment == 5 {
				root = i
				break
			}
		}
		toothFaces := tooth.Faces()
		if root < 3 || len(toothFaces) != lp.walls+2 || lp.walls < 2*root {
			return nil, fmt.Errorf("%w: loft/cylinder tooth has no paired exterior faces", ErrUnsupported)
		}
		for j := range root {
			for side := range 2 {
				f := toothFaces[2+2*j+side]
				if len(f.origins) == 0 {
					return nil, fmt.Errorf("%w: loft/cylinder tooth face lost its origin", ErrUnsupported)
				}
				origin := f.origins[0]
				faces[origin] = append(faces[origin], loftCylinderChamferToothFace{
					body: tooth, loft: lp, root: root, cell: j, bound: lp.proof.FacetDeparture,
					scale: proofarith.FloatRat(original.xform.Basis().EZ.Z),
				})
			}
		}
	}
	return faces, nil
}
