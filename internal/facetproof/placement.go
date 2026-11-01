package facetproof

import (
	"context"
	"fmt"
	"math"
	"sync"
	"unsafe"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// PlacedMesh carries the held geometry and bounds after a faceted placement.
type PlacedMesh struct {
	Verts       []r3.Vec
	Tris        [][3]int
	VertexBound []float64
	MeshBound   float64
	VolSymDiff  float64
	AreaSlack   float64
	// EmbeddingReach records only a new accepted positive-reach repair.
	EmbeddingReach *meshbool.EmbeddingReach
}

// placeParallelMin is the vertex count from which PlaceMesh computes its exact
// images on several goroutines.
const placeParallelMin = 1024

// PlaceMesh rounds each exact affine image to its nearest float, keeps its
// held facets embedded, and charges the stored basis's exact stretch and
// determinant before each vertex's own rounding and embedding correction.
// Large meshes compute the vertex images on several goroutines; the budget
// steps and the first non-representable vertex are read in vertex order on the
// calling goroutine.
func PlaceMesh(ctx context.Context, budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int,
	vertexBound []float64, volSymDiff, areaSlack, stretch, volumeCharge float64,
	delta r3.Transform) (PlacedMesh, error) {
	return placeMesh(ctx, budget, verts, tris, vertexBound, volSymDiff, areaSlack, stretch, volumeCharge, delta,
		orderedwork.EachWorkers(len(verts), placeParallelMin))
}

func placeMesh(ctx context.Context, budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int,
	vertexBound []float64, volSymDiff, areaSlack, stretch, volumeCharge float64,
	delta r3.Transform, workers int) (PlacedMesh, error) {
	return placeMeshGap(ctx, budget, verts, tris, vertexBound, volSymDiff, areaSlack, stretch, volumeCharge,
		delta, workers, nil)
}

// PlaceMeshReplay uses serial placement and the shared counter for every proof phase.
func PlaceMeshReplay(ctx context.Context, budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int,
	vertexBound []float64, volSymDiff, areaSlack, stretch, volumeCharge float64, delta r3.Transform) (PlacedMesh, error) {
	if budget == nil {
		return PlacedMesh{}, fmt.Errorf("%w: bounded placement requires a budget", decaderr.ErrUnsupported)
	}
	return placeMeshGapBudget(ctx, budget, verts, tris, vertexBound, volSymDiff, areaSlack, stretch, volumeCharge, delta, 1, nil, true)
}

// PlaceMeshDenoting places verts by d exactly as PlaceMesh does, for a source
// that the result denotes under an exactly orthogonal map Q with translation
// t, where linearGap bounds the entrywise absolute sum of Q - D and
// translationGap bounds the entrywise absolute sum of t - tD (D, tD being d's
// basis and translation read exactly). Q keeps length, area and volume, so the
// stretch and determinant charges are zero. Each vertex instead pays
// linearGap * max|v_j| + translationGap, because a true point within beta of
// the held source vertex v lies within |p - v| + |(Q - D)v| + |t - tD| of D v
// + tD before the vertex's own rounding and embedding correction.
//
// linearGap and translationGap must be finite and non-negative.
func PlaceMeshDenoting(ctx context.Context, budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int,
	vertexBound []float64, volSymDiff, areaSlack float64, d r3.Transform,
	linearGap, translationGap float64) (PlacedMesh, error) {
	if proofbound.IsNonFinite(linearGap) || proofbound.IsNonFinite(translationGap) ||
		linearGap < 0 || translationGap < 0 {
		return PlacedMesh{}, fmt.Errorf("%w: placement gap is not a finite bound", decaderr.ErrUnsupported)
	}
	return placeMeshGap(ctx, budget, verts, tris, vertexBound, volSymDiff, areaSlack, 0, 0, d,
		orderedwork.EachWorkers(len(verts), placeParallelMin), &denotingGap{linear: linearGap, translation: translationGap})
}

// denotingGap is the distance between the float placement and the exactly
// orthogonal map the placed mesh denotes.
type denotingGap struct{ linear, translation float64 }

func placeMeshGap(ctx context.Context, budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int,
	vertexBound []float64, volSymDiff, areaSlack, stretch, volumeCharge float64,
	delta r3.Transform, workers int, gap *denotingGap) (PlacedMesh, error) {
	return placeMeshGapBudget(ctx, budget, verts, tris, vertexBound, volSymDiff, areaSlack, stretch, volumeCharge, delta, workers, gap, false)
}

func placeMeshGapBudget(ctx context.Context, budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int,
	vertexBound []float64, volSymDiff, areaSlack, stretch, volumeCharge float64,
	delta r3.Transform, workers int, gap *denotingGap, replay bool) (PlacedMesh, error) {
	if proofbound.IsNonFinite(volSymDiff) || proofbound.IsNonFinite(areaSlack) ||
		proofbound.IsNonFinite(stretch) || proofbound.IsNonFinite(volumeCharge) ||
		volSymDiff < 0 || areaSlack < 0 || stretch < 0 || volumeCharge < 0 ||
		len(verts) != len(vertexBound) {
		return PlacedMesh{}, fmt.Errorf("%w: placement has no finite source proof", decaderr.ErrUnsupported)
	}
	if replay {
		if len(verts) > 65536 || len(tris) > 65536 {
			return PlacedMesh{}, fmt.Errorf("%w: bounded placement facet cap exceeded", decaderr.ErrUnsupported)
		}
		if err := budget.Reserve(12*uint64(len(verts)) + 8*uint64(len(tris)) + 10); err != nil {
			return PlacedMesh{}, err
		}
		if _, err := budget.StepN(8*uint64(len(verts)) + 6*uint64(len(tris))); err != nil {
			return PlacedMesh{}, err
		}
	}
	if replay {
		for _, size := range []uint64{uint64(unsafe.Sizeof(r3.Vec{})), uint64(unsafe.Sizeof(float64(0))), uint64(unsafe.Sizeof(float64(0))), uint64(unsafe.Sizeof(proof.Xpt{})), uint64(unsafe.Sizeof(false)), uint64(unsafe.Sizeof(float64(0)))} {
			if err := budget.ReserveBytesMul(uint64(len(verts)), size); err != nil {
				return PlacedMesh{}, err
			}
		}
	}
	next := PlacedMesh{Verts: make([]r3.Vec, len(verts)), Tris: tris}
	tr := delta.Translation()
	maxTrans := math.Max(math.Abs(tr.X), math.Max(math.Abs(tr.Y), math.Abs(tr.Z)))
	vertexAllow := make([]float64, len(verts))
	extra := make([]float64, len(verts))
	exact := make([]proof.Xpt, len(verts))
	// denotes[i] records that the placed vertex is its exact image. The
	// embedding check below reads the same predicate on the same two values
	// for its Moved marks, so it takes this answer instead of deciding it again.
	denotes := make([]bool, len(verts))
	basis := delta.Basis()
	// An image panics on a non-finite coordinate, so such a placement takes
	// the vertex-order path.
	if workers >= 2 && (!proofbound.AllFinite(verts) ||
		!proofbound.AllFinite([]r3.Vec{basis.EX, basis.EY, basis.EZ, tr})) {
		workers = 1
	}
	place := func(i int) error {
		v := verts[i]
		maxAbs := math.Max(math.Abs(v.X), math.Max(math.Abs(v.Y), math.Abs(v.Z)))
		vertexAllow[i] = proofbound.RigidRoundAllow(maxAbs, maxTrans)
		if gap != nil {
			extra[i] = proofbound.AbsSumUpper(proofbound.ProductUpper(gap.linear, maxAbs), gap.translation)
		}
		if replay {
			var err error
			exact[i], err = proof.ReplayExactAffineImage(ctx, basis.EX, basis.EY, basis.EZ, tr, v, replayExactMeter{budget})
			if err != nil {
				return err
			}
			next.Verts[i], err = proof.ReplayXptVec(ctx, exact[i], replayExactMeter{budget})
			if err != nil {
				return err
			}
		} else {
			exact[i] = proof.ExactAffineImage(basis.EX, basis.EY, basis.EZ, tr, v)
			next.Verts[i] = exact[i].Vec()
		}
		// An image that is itself a float, as a whole-number translation of whole-number coordinates is, has no
		// rounding to allow for: the placed vertex is the exact image.
		if proofbound.FiniteVec(next.Verts[i]) {
			same := false
			if replay {
				var err error
				same, err = proof.ReplayXptDenotes(ctx, exact[i], next.Verts[i], replayExactMeter{budget})
				if err != nil {
					return err
				}
			} else {
				same = proof.XptDenotes(exact[i], next.Verts[i])
			}
			if same {
				vertexAllow[i] = 0
				denotes[i] = true
			}
		}
		return nil
	}
	if workers >= 2 {
		if _, err := orderedwork.Each(len(verts), workers, 512, budget.Err, place); err != nil {
			return PlacedMesh{}, err
		}
	}
	for i := range verts {
		if err := budget.Step(); err != nil {
			return PlacedMesh{}, err
		}
		if workers < 2 {
			if err := place(i); err != nil {
				return PlacedMesh{}, err
			}
		}
		if !proofbound.FiniteVec(next.Verts[i]) {
			return PlacedMesh{}, fmt.Errorf("%w: a placed vertex is not representable", decaderr.ErrUnsupported)
		}
	}
	if err := budget.Err(); err != nil {
		return PlacedMesh{}, err
	}
	if delta.IsReflection() {
		if replay {
			if err := budget.ReserveBytesMul(uint64(len(tris)), uint64(unsafe.Sizeof([3]int{}))); err != nil {
				return PlacedMesh{}, err
			}
		}
		next.Tris = make([][3]int, len(tris))
		for i, t := range tris {
			if err := budget.Step(); err != nil {
				return PlacedMesh{}, err
			}
			next.Tris[i] = [3]int{t[0], t[2], t[1]}
		}
	}
	// The source area reads only verts and tris, which nothing writes during
	// the call, so it runs beside the embedding check and gives the same float
	// sum in the same facet order as a serial call would.
	var heldArea float64
	var areaDone sync.WaitGroup
	if replay {
		heldArea = MeshAreaUpper(verts, tris)
	} else {
		areaDone.Go(func() { heldArea = MeshAreaUpper(verts, tris) })
	}
	var moved []float64
	var repair *meshbool.EmbeddingReach
	var err error
	if replay {
		moved, repair, err = keepPlacedEmbeddedWithAxesReplay(ctx, exact, next.Verts, next.Tris, [3]r3.Vec{basis.EX, basis.EY, basis.EZ}, denotes, budget)
	} else {
		moved, repair, err = keepPlacedEmbeddedWithAxes(ctx, exact, next.Verts, next.Tris, [3]r3.Vec{basis.EX, basis.EY, basis.EZ}, denotes)
	}
	if err != nil {
		areaDone.Wait()
		return PlacedMesh{}, err
	}
	next.EmbeddingReach = repair
	allow := 0.0
	next.VertexBound = make([]float64, len(vertexBound))
	lengthStretch := proofbound.AbsSumUpper(1, stretch)
	for i, beta := range vertexBound {
		a := vertexAllow[i]
		if moved != nil {
			a = math.Max(a, moved[i])
		}
		if gap != nil {
			a = proofbound.AbsSumUpper(a, extra[i])
		}
		allow = math.Max(allow, a)
		next.VertexBound[i] = proofbound.AbsSumUpper(proofbound.ProductUpper(lengthStretch, beta), a)
	}
	next.MeshBound = FacetBoundMax(next.Tris, next.VertexBound)
	var areaUpper float64
	var allows []float64
	if replay {
		areaUpper, allows, err = proofbound.PerturbedAreaUpperAllowsReplay(ctx, budget, next.Verts, next.Tris, allow)
	} else {
		areaUpper, allows, err = proofbound.PerturbedAreaUpperAllowsContext(ctx, next.Verts, next.Tris, allow)
	}
	areaDone.Wait()
	if err != nil {
		return PlacedMesh{}, err
	}
	next.VolSymDiff = proofbound.AbsSumUpper(
		proofbound.ProductUpper(proofbound.AbsSumUpper(1, volumeCharge), volSymDiff),
		proofbound.SweptVolumeAllow(allow, areaUpper))
	areaRound := 0.0
	for i := range next.Tris {
		if err := budget.Step(); err != nil {
			return PlacedMesh{}, err
		}
		areaRound = proofbound.AbsSumUpper(areaRound, allows[i])
	}
	// True and held tangent planes can gain opposite area changes under the
	// same nearly orthonormal map. Charge each against its own area: if their
	// old difference is E and held area is H, the new difference is at most
	// (1+e)E + 2eH before held-coordinate rounding.
	next.AreaSlack = proofbound.AbsSumUpper(areaSlack,
		proofbound.ProductUpper(stretch, areaSlack),
		proofbound.ProductUpper(2, proofbound.ProductUpper(stretch, heldArea)), areaRound)
	if proofbound.IsNonFinite(next.MeshBound) || proofbound.IsNonFinite(next.VolSymDiff) ||
		proofbound.IsNonFinite(next.AreaSlack) {
		return PlacedMesh{}, fmt.Errorf("%w: placement has no finite image proof", decaderr.ErrUnsupported)
	}
	return next, nil
}

// FacetBoundMax is the largest held facet bound over the mesh.
func FacetBoundMax(tris [][3]int, beta []float64) float64 {
	out := 0.0
	for _, t := range tris {
		out = max(out, beta[t[0]], beta[t[1]], beta[t[2]])
	}
	return out
}
