package loftcylinder

import (
	"context"
	"fmt"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// ChamferToothFace is one authenticated live tooth face's transient proof input.
type ChamferToothFace struct {
	Body       *topo.Body
	Loft       loftrecord.Payload
	Root, Cell int
	Bound      float64
	Scale      *big.Rat
}

// ChamferMiddleInput binds the retained facets to their source and old shell.
type ChamferMiddleInput struct {
	Mesh                            *facetedrecord.JoinedMesh
	Ideal                           []proofarith.Xpt
	TriCount                        int
	ClipBound                       float64
	Faces                           map[provenance.Ref][]ChamferToothFace
	Old                             *loftmesh.LoftAuditData
	OldBoxes                        [][2]r3.Vec
	Z0, Z1, BoreRadius, BlankRadius float64
}

// ChamferMiddleState shares the supplied grid and per-tooth memo across facets.
type ChamferMiddleState struct {
	Grid         *loftmesh.ChamferBoxGrid
	parity       *meshbool.ParityMesh
	toothParity  map[*topo.Body]*meshbool.ParityMesh
	toothCircles map[*topo.Body]SourceCircle
	toothRoots   map[*topo.Body]*loftmesh.ChamferHeldRootMemo
}

// NewChamferMiddleState creates one worker's shared facet state.
func NewChamferMiddleState(grid *loftmesh.ChamferBoxGrid, parity *meshbool.ParityMesh) *ChamferMiddleState {
	return &ChamferMiddleState{
		Grid: grid, parity: parity,
		toothParity:  make(map[*topo.Body]*meshbool.ParityMesh),
		toothCircles: make(map[*topo.Body]SourceCircle),
		toothRoots:   make(map[*topo.Body]*loftmesh.ChamferHeldRootMemo),
	}
}

// ChamferMiddleFacet proves one retained middle facet reaches the source cut.
func ChamferMiddleFacet(ctx context.Context, input ChamferMiddleInput, i int, st *ChamferMiddleState) error {
	mesh, ideal := input.Mesh, input.Ideal
	group := mesh.Src[i]
	if group < 0 || group >= len(mesh.Groups) || len(mesh.Groups[group].Origins) == 0 {
		return fmt.Errorf("%w: retained middle facet lost its source group", decaderr.ErrUnsupported)
	}
	origin := mesh.Groups[group].Origins[0]
	var base [3]proofarith.Xpt
	for j, index := range mesh.Tris[i] {
		if index < 0 || index >= len(ideal) {
			return fmt.Errorf("%w: retained middle facet has an invalid vertex", decaderr.ErrUnsupported)
		}
		base[j] = ideal[index]
		z := meshbool.RatCoordOf(base[j], 2)
		if z.Cmp(proofarith.FloatRat(input.Z0)) <= 0 ||
			z.Cmp(proofarith.FloatRat(input.Z1)) >= 0 {
			return fmt.Errorf("%w: retained middle facet reaches a source cap", decaderr.ErrUnsupported)
		}
	}
	last := "no inward source witness"
	var corners [6]loftmesh.ChamferCorner
	baseRounded := false
	for _, distance := range [...]float64{0.08, 0.06, 0.04, 0.02, 0.01} {
		q, shift, err := loftmesh.ChamferMiddleOffset(base, distance, input.ClipBound)
		if err != nil {
			return err
		}
		if !loftmesh.ChamferMiddleBeyondBore(q, input.BoreRadius) {
			last = "inward patch reaches the source bore"
			continue
		}
		for k := range q {
			corners[3+k] = loftmesh.ChamferCornerOf(q[k])
		}
		qBox, qBoxed := loftmesh.ChamferCornersBox(corners[3], corners[4], corners[5])
		if !loftmesh.ChamferMiddleBlank(q, input.BlankRadius) {
			candidates := input.Faces[origin]
			if len(candidates) == 0 {
				last = "source face has no authenticated tooth"
				continue
			}
			certified := false
			for _, face := range candidates {
				parity, ok := st.toothParity[face.Body]
				if !ok {
					parity = meshbool.NewParityMesh(face.Loft.Verts, face.Loft.Tris)
					st.toothParity[face.Body] = parity
				}
				if err := chamferMiddleToothSource(ctx, q, qBox, qBoxed, face,
					parity, input.BlankRadius, st.toothCircles, st.toothRoots); err != nil {
					last = err.Error()
					continue
				}
				certified = true
				break
			}
			if !certified {
				continue
			}
		}
		if !baseRounded {
			for k := range base {
				corners[k] = loftmesh.ChamferCornerOf(base[k])
			}
			baseRounded = true
		}
		if err := loftmesh.ChamferMiddlePrism(ctx, input.TriCount, input.Old, input.OldBoxes, st.Grid,
			st.parity, base, q, shift, corners); err != nil {
			if ctx.Err() != nil {
				return ctx.Err()
			}
			last = err.Error()
			continue
		}
		return nil
	}
	return fmt.Errorf("%w: retained middle facet %d (origin %v) has no certified inward collar: %s",
		decaderr.ErrUnsupported, i, origin, last)
}

// A whole inward patch is in the true Loft tooth when it misses every moving
// wall by its matched departure and one held-tooth point is inside. If only
// hidden root walls meet it, use the exact blank disk for that part instead.
func chamferMiddleToothSource(ctx context.Context, q [3]proofarith.Xpt,
	qBox [2]r3.Vec, boxed bool,
	face ChamferToothFace, parity *meshbool.ParityMesh,
	blankRadius float64, circles map[*topo.Body]SourceCircle,
	roots map[*topo.Body]*loftmesh.ChamferHeldRootMemo) error {
	lp := face.Loft
	if lp.Walls <= 0 || lp.Walls > len(lp.Tris) || face.Root < 3 ||
		face.Scale == nil || face.Scale.Sign() <= 0 ||
		2*face.Root >= lp.Walls || face.Bound < 0 || proofbound.IsNonFinite(face.Bound) {
		return fmt.Errorf("%w: inward patch lacks a bounded Loft wall", decaderr.ErrUnsupported)
	}
	fullClear := true
	for j := range lp.Walls {
		if j%64 == 0 && ctx.Err() != nil {
			return ctx.Err()
		}
		tri := lp.Tris[j]
		if boxed && loftmesh.ChamferBoxGapAbove(qBox, meshbool.TriBox(lp.Verts, tri), face.Bound) {
			continue
		}
		other := [3]proofarith.Xpt{
			proofarith.XptOf(lp.Verts[tri[0]]),
			proofarith.XptOf(lp.Verts[tri[1]]),
			proofarith.XptOf(lp.Verts[tri[2]]),
		}
		if loftmesh.ChamferTriangleGapSplit(q, other, face.Bound, 2) ||
			meshbool.DisjointTriDistanceAbove(q, other, face.Bound) {
			continue
		}
		if j < 2*face.Root {
			return fmt.Errorf("%w: inward patch approaches exterior tooth facet %d", decaderr.ErrUnsupported, j)
		}
		fullClear = false
	}
	if fullClear {
		witness := meshbool.XCentroid(q[0], q[1], q[2])
		inside, on, err := meshbool.MeshParityAllContext(ctx, witness, parity)
		if err != nil {
			return err
		}
		if inside && !on {
			return nil
		}
	}
	// The original root arc is hidden by the blank over this middle slab.
	// Both matched held and true root patches must remain in the convex disk;
	// their entire same-height homotopy then stays in that disk too.
	circle, ok := circles[face.Body]
	if !ok {
		var err error
		circle, err = SourceCircleOf(lp, blankRadius)
		if err != nil {
			return err
		}
		circles[face.Body] = circle
	}
	zLo, zHi := meshbool.RatCoordOf(q[0], 2), meshbool.RatCoordOf(q[0], 2)
	for _, p := range q[1:] {
		z := meshbool.RatCoordOf(p, 2)
		if z.Cmp(zLo) < 0 {
			zLo = z
		}
		if z.Cmp(zHi) > 0 {
			zHi = z
		}
	}
	mappedLo := new(big.Rat).Quo(zLo, face.Scale)
	mappedHi := new(big.Rat).Quo(zHi, face.Scale)
	memo := roots[face.Body]
	if memo == nil {
		memo = new(loftmesh.ChamferHeldRootMemo)
		roots[face.Body] = memo
	}
	if mappedLo.Cmp(circle.SafeBottom) <= 0 || mappedHi.Cmp(circle.SafeTop) >= 0 ||
		!memo.Inside(lp.Verts, lp.Tris, lp.Walls, face.Root, mappedLo, mappedHi, blankRadius) {
		return fmt.Errorf("%w: inward patch has no hidden root-wall interval", decaderr.ErrUnsupported)
	}
	r := proofarith.FloatRat(blankRadius)
	r2 := new(big.Rat).Mul(r, r)
	outside := 0
	for _, p := range q {
		x, y := meshbool.RatCoordOf(p, 0), meshbool.RatCoordOf(p, 1)
		diff := new(big.Rat).Add(new(big.Rat).Mul(x, x), new(big.Rat).Mul(y, y))
		if diff.Cmp(r2) <= 0 {
			continue
		}
		outside++
		inside, on, err := meshbool.MeshParityAllContext(ctx, p, parity)
		if err != nil {
			return err
		}
		if !inside || on {
			return fmt.Errorf("%w: inward patch exterior vertex leaves its held tooth", decaderr.ErrUnsupported)
		}
	}
	if outside == 0 {
		return fmt.Errorf("%w: inward patch root split has no exterior witness", decaderr.ErrUnsupported)
	}
	// Each connected part of a triangle outside a centered convex disk reaches
	// an outside vertex: from x choose a vertex v with x·v >= |x|², and the
	// segment x--v stays outside. No source wall meets that part, so its
	// inside vertex fixes occupancy. The remaining patch is inside the blank.
	return nil
}

// chamferMiddleSidePrep is the read-only input of the retained middle-side
// proof: the validated payload, the tooth faces and the old shell's audit
// data. The grid is the one mutable part; a worker uses a Fork of it.
type chamferMiddleSidePrep struct {
	fp     facetedrecord.Payload
	middle [2]int
	grid   *loftmesh.ChamferBoxGrid
	proof  ChamferMiddleInput
}

func newChamferMiddleSidePrep(fp facetedrecord.Payload, mesh *facetedrecord.JoinedMesh, source ChamferSource,
	ideal []proofarith.Xpt, middle [2]int, clipBound float64) (*chamferMiddleSidePrep, error) {
	if middle[0] < 0 || middle[1] > len(mesh.Tris) || middle[0] >= middle[1] ||
		len(fp.Src) != len(fp.Tris) || proofbound.IsNonFinite(clipBound) {
		return nil, fmt.Errorf("%w: cap chamfer has no retained middle side", decaderr.ErrUnsupported)
	}
	faces, err := chamferToothFaces(source)
	if err != nil {
		return nil, err
	}
	old := loftmesh.NewLoftAuditData(fp.Verts, fp.Tris)
	oldBoxes := make([][2]r3.Vec, len(fp.Tris))
	for i, tri := range fp.Tris {
		oldBoxes[i] = meshbool.TriBox(fp.Verts, tri)
	}
	oldGrid, err := loftmesh.ChamferBoxGridOf(oldBoxes, source.Z0, source.Z1)
	if err != nil {
		return nil, err
	}
	// A whole old boundary component cannot hide inside a middle collar:
	// every old vertex is on a cap plane, outside every strict middle prism.
	for _, p := range fp.Verts {
		if p.Z != source.Z0 && p.Z != source.Z1 {
			return nil, fmt.Errorf("%w: joined source has an interior-height vertex", decaderr.ErrUnsupported)
		}
	}
	prep := &chamferMiddleSidePrep{
		fp: fp, middle: middle, grid: oldGrid,
		proof: ChamferMiddleInput{
			Mesh: mesh, Ideal: ideal, TriCount: len(fp.Tris), ClipBound: clipBound,
			Faces: faces, Old: old, OldBoxes: oldBoxes,
			Z0: source.Z0, Z1: source.Z1,
			BoreRadius: source.BoreRadius, BlankRadius: source.BlankRadius,
		},
	}
	return prep, nil
}

// A retained middle facet is an exact subset of an old held wall. An inward
// horizontal collar ending inside the source solid gives every point of that
// facet a source-cut boundary point within the collar's length.
func chamferMiddleSideReach(ctx context.Context, fp facetedrecord.Payload,
	mesh *facetedrecord.JoinedMesh, source ChamferSource, ideal []proofarith.Xpt,
	middle [2]int, clipBound float64) error {
	prep, err := newChamferMiddleSidePrep(fp, mesh, source, ideal, middle, clipBound)
	if err != nil {
		return err
	}
	return prep.serialFrom(ctx, middle[0])
}

// serialFrom runs the facets first..middle[1] in order on the real grid.
func (prep *chamferMiddleSidePrep) serialFrom(ctx context.Context, first int) error {
	st := NewChamferMiddleState(prep.grid,
		meshbool.NewParityMesh(prep.fp.Verts, prep.fp.Tris))
	for i := first; i < prep.middle[1]; i++ {
		if i%32 == 0 && ctx.Err() != nil {
			return ctx.Err()
		}
		if err := ChamferMiddleFacet(ctx, prep.proof, i, st); err != nil {
			return err
		}
	}
	return nil
}

// Union copied these precise Loft face triangles, and Cut retained their
// groups. Match by authenticated origin instead of interpreting a role string
// on a faceted result that may coalesce several public edges.
func chamferToothFaces(source ChamferSource) (map[provenance.Ref][]ChamferToothFace, error) {
	if source.Blank == nil || len(source.Teeth) == 0 ||
		len(source.OriginalTeeth) != len(source.Teeth) || source.BlankRadius <= 0 {
		return nil, fmt.Errorf("%w: loft/cylinder chamfer lost its source operands", decaderr.ErrUnsupported)
	}
	faces := make(map[provenance.Ref][]ChamferToothFace)
	for toothIndex, tooth := range source.Teeth {
		if tooth == nil || source.OriginalTeeth[toothIndex] == nil {
			return nil, fmt.Errorf("%w: loft/cylinder tooth source is absent", decaderr.ErrUnsupported)
		}
		lp, ok := tooth.Payload.(loftrecord.Payload)
		original, originalOK := source.OriginalTeeth[toothIndex].Payload.(loftrecord.Payload)
		if !ok || !lp.AuthenticatedReconstruction || lp.SourceIdentity == nil ||
			!originalOK || original.SourceIdentity != lp.SourceIdentity ||
			lp.Proof.FacetDeparture < 0 || proofbound.IsNonFinite(lp.Proof.FacetDeparture) ||
			len(lp.Segment) != 1 {
			return nil, fmt.Errorf("%w: loft/cylinder tooth lost its certified surface", decaderr.ErrUnsupported)
		}
		root := -1
		for i, segment := range lp.Segment[0] {
			if segment == 5 {
				root = i
				break
			}
		}
		toothFaces := topobuild.Faces(tooth)
		if root < 3 || len(toothFaces) != lp.Walls+2 || lp.Walls < 2*root {
			return nil, fmt.Errorf("%w: loft/cylinder tooth has no paired exterior faces", decaderr.ErrUnsupported)
		}
		for j := range root {
			for side := range 2 {
				f := toothFaces[2+2*j+side]
				if len(f.Origins) == 0 {
					return nil, fmt.Errorf("%w: loft/cylinder tooth face lost its origin", decaderr.ErrUnsupported)
				}
				origin := f.Origins[0]
				faces[origin] = append(faces[origin], ChamferToothFace{
					Body: tooth, Loft: lp, Root: root, Cell: j, Bound: lp.Proof.FacetDeparture,
					Scale: proofarith.FloatRat(original.Xform.Basis().EZ.Z),
				})
			}
		}
	}
	return faces, nil
}
