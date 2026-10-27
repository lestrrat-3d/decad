package facetproof

import (
	"context"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/gaptol"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	"github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// serialPlaceMeshGap is placeMeshGap as a single serial pass: the embedding
// check decides every vertex's Moved mark from the exact predicate itself,
// and the source area is summed after the embedding check on the calling
// goroutine. TestPlaceMeshMatchesSerialPlacement compares the two.
func serialPlaceMeshGap(ctx context.Context, budget *proofbound.WorkBudget, verts []r3.Vec, tris [][3]int,
	vertexBound []float64, volSymDiff, areaSlack, stretch, volumeCharge float64,
	delta r3.Transform, workers int, gap *denotingGap) (PlacedMesh, error) {
	if proofbound.IsNonFinite(volSymDiff) || proofbound.IsNonFinite(areaSlack) ||
		proofbound.IsNonFinite(stretch) || proofbound.IsNonFinite(volumeCharge) ||
		volSymDiff < 0 || areaSlack < 0 || stretch < 0 || volumeCharge < 0 ||
		len(verts) != len(vertexBound) {
		return PlacedMesh{}, fmt.Errorf("%w: placement has no finite source proof", decaderr.ErrUnsupported)
	}
	next := PlacedMesh{Verts: make([]r3.Vec, len(verts)), Tris: tris}
	tr := delta.Translation()
	maxTrans := math.Max(math.Abs(tr.X), math.Max(math.Abs(tr.Y), math.Abs(tr.Z)))
	vertexAllow := make([]float64, len(verts))
	extra := make([]float64, len(verts))
	exact := make([]proof.Xpt, len(verts))
	basis := delta.Basis()
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
		exact[i] = proof.ExactAffineImage(basis.EX, basis.EY, basis.EZ, tr, v)
		next.Verts[i] = exact[i].Vec()
		if proofbound.FiniteVec(next.Verts[i]) && proof.XptDenotes(exact[i], next.Verts[i]) {
			vertexAllow[i] = 0
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
			_ = place(i)
		}
		if !proofbound.FiniteVec(next.Verts[i]) {
			return PlacedMesh{}, fmt.Errorf("%w: a placed vertex is not representable", decaderr.ErrUnsupported)
		}
	}
	if err := budget.Err(); err != nil {
		return PlacedMesh{}, err
	}
	if delta.IsReflection() {
		next.Tris = make([][3]int, len(tris))
		for i, t := range tris {
			if err := budget.Step(); err != nil {
				return PlacedMesh{}, err
			}
			next.Tris[i] = [3]int{t[0], t[2], t[1]}
		}
	}
	moved, repair, err := keepPlacedEmbeddedWithAxes(ctx, exact, next.Verts, next.Tris,
		[3]r3.Vec{basis.EX, basis.EY, basis.EZ}, nil)
	if err != nil {
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
	areaUpper, allows, err := proofbound.PerturbedAreaUpperAllowsContext(ctx, next.Verts, next.Tris, allow)
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
	heldArea := MeshAreaUpper(verts, tris)
	next.AreaSlack = proofbound.AbsSumUpper(areaSlack,
		proofbound.ProductUpper(stretch, areaSlack),
		proofbound.ProductUpper(2, proofbound.ProductUpper(stretch, heldArea)), areaRound)
	if proofbound.IsNonFinite(next.MeshBound) || proofbound.IsNonFinite(next.VolSymDiff) ||
		proofbound.IsNonFinite(next.AreaSlack) {
		return PlacedMesh{}, fmt.Errorf("%w: placement has no finite image proof", decaderr.ErrUnsupported)
	}
	return next, nil
}

// differentialSphere is a closed latitude-longitude sphere of radius 10 about
// center, with 2*lon*(lat-1) facets. 40 by 64 gives 4,992 facets, enough for
// the embedding check's parallel first scan.
func differentialSphere(center r3.Vec, lat, lon int) ([]r3.Vec, [][3]int) {
	verts := []r3.Vec{center.Add(r3.Vec{Z: 10}), center.Add(r3.Vec{Z: -10})}
	for i := 1; i < lat; i++ {
		theta := math.Pi * float64(i) / float64(lat)
		for j := range lon {
			phi := 2 * math.Pi * float64(j) / float64(lon)
			verts = append(verts, center.Add(r3.Vec{
				X: 10 * math.Sin(theta) * math.Cos(phi),
				Y: 10 * math.Sin(theta) * math.Sin(phi),
				Z: 10 * math.Cos(theta),
			}))
		}
	}
	ring := func(i, j int) int { return 2 + (i-1)*lon + (j % lon) }
	var tris [][3]int
	for j := range lon {
		tris = append(tris, [3]int{0, ring(1, j), ring(1, j+1)})
		tris = append(tris, [3]int{1, ring(lat-1, j+1), ring(lat-1, j)})
	}
	for i := 1; i < lat-1; i++ {
		for j := range lon {
			a, b := ring(i, j), ring(i, j+1)
			c, d := ring(i+1, j), ring(i+1, j+1)
			tris = append(tris, [3]int{a, c, d}, [3]int{a, d, b})
		}
	}
	return verts, tris
}

// differentialFold reads testdata/placement_fold.json, whose quarter turn
// about Z folds four facets that only a positive reach clears.
func differentialFold(t *testing.T) ([]r3.Vec, [][3]int) {
	t.Helper()
	raw, err := os.ReadFile("testdata/placement_fold.json")
	require.NoError(t, err)
	var d struct {
		V [][3]float64 `json:"V"`
		T [][3]int     `json:"T"`
	}
	require.NoError(t, json.Unmarshal(raw, &d))
	verts := make([]r3.Vec, len(d.V))
	for i, v := range d.V {
		verts[i] = r3.Vec{X: v[0], Y: v[1], Z: v[2]}
	}
	return verts, d.T
}

// requireSameBits fails at the first entry whose bits differ, naming only
// that index and its two values.
func requireSameBits(t *testing.T, what string, want, got []float64) {
	t.Helper()
	require.Len(t, got, len(want), what)
	for i := range want {
		if math.Float64bits(want[i]) != math.Float64bits(got[i]) {
			require.Failf(t, what+" differs", "index %d: want %v, got %v", i, want[i], got[i])
		}
	}
}

func requireSamePlacement(t *testing.T, want, got PlacedMesh) {
	t.Helper()
	require.Len(t, got.Verts, len(want.Verts))
	for i := range want.Verts {
		w, g := want.Verts[i], got.Verts[i]
		if math.Float64bits(w.X) != math.Float64bits(g.X) || math.Float64bits(w.Y) != math.Float64bits(g.Y) ||
			math.Float64bits(w.Z) != math.Float64bits(g.Z) {
			require.Failf(t, "placed vertex differs", "vertex %d: want %v, got %v", i, w, g)
		}
	}
	require.Len(t, got.Tris, len(want.Tris))
	for i := range want.Tris {
		if want.Tris[i] != got.Tris[i] {
			require.Failf(t, "placed facet differs", "facet %d: want %v, got %v", i, want.Tris[i], got.Tris[i])
		}
	}
	requireSameBits(t, "vertex bound", want.VertexBound, got.VertexBound)
	requireSameBits(t, "mesh bounds", []float64{want.MeshBound, want.VolSymDiff, want.AreaSlack},
		[]float64{got.MeshBound, got.VolSymDiff, got.AreaSlack})
	require.Equal(t, want.EmbeddingReach == nil, got.EmbeddingReach == nil, "embedding reach presence")
	if want.EmbeddingReach != nil {
		requireSameBits(t, "embedding reach",
			[]float64{want.EmbeddingReach.DisplacementBound, want.EmbeddingReach.AllowedReach},
			[]float64{got.EmbeddingReach.DisplacementBound, got.EmbeddingReach.AllowedReach})
	}
}

// TestPlaceMeshMatchesSerialPlacement compares PlaceMesh, which reads the
// placement's own XptDenotes answers for the embedding check's Moved marks and
// sums the source area beside that check, with the serial pass that decides
// both itself. Every held coordinate, bound, repair summary and error is the
// same, on a sphere large enough for the parallel scans, under rotations and a
// reflection, and on the fold that only a positive reach clears.
func TestPlaceMeshMatchesSerialPlacement(t *testing.T) {
	t.Parallel()
	turn := func(axis r3.Vec, degrees float64) r3.Transform {
		rot, err := r3.Rotation(axis, units.Degrees(degrees))
		require.NoError(t, err)
		return rot
	}
	mirror, err := r3.NewFrame(r3.NewVec(0.3, 0, 0), r3.NewVec(0, 1, 0), r3.NewVec(0, 0, 1))
	require.NoError(t, err)
	reflect, err := r3.Reflection(mirror)
	require.NoError(t, err)
	mirrored, err := turn(r3.NewVec(1, 2, 3), 17).Then(reflect)
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(0.1, -0.7, 1e-3))
	require.NoError(t, err)
	shifted, err := turn(r3.Vec{X: 1}, 9).Then(shift)
	require.NoError(t, err)

	sphereVerts, sphereTris := differentialSphere(r3.NewVec(12.5, -3.25, 0.75), 40, 64)
	require.GreaterOrEqual(t, len(sphereTris), 4096, "the sphere reaches the parallel scan")
	foldVerts, foldTris := differentialFold(t)
	foldTurn := turn(r3.Vec{Z: 1}, 90)

	type input struct {
		name  string
		reach float64
		verts []r3.Vec
		tris  [][3]int
		delta r3.Transform
	}
	var inputs []input
	for _, degrees := range []float64{9, 37, 90, 211} {
		inputs = append(inputs, input{name: fmt.Sprintf("sphere x %v", degrees),
			verts: sphereVerts, tris: sphereTris, delta: turn(r3.Vec{X: 1}, degrees)})
	}
	inputs = append(inputs,
		input{name: "sphere oblique", verts: sphereVerts, tris: sphereTris,
			delta: turn(r3.NewVec(1, 2, 3), 23)},
		input{name: "sphere mirrored", verts: sphereVerts, tris: sphereTris, delta: mirrored},
		input{name: "sphere shifted", verts: sphereVerts, tris: sphereTris, delta: shifted},
		input{name: "fold with reach", reach: 1e-3, verts: foldVerts, tris: foldTris,
			delta: foldTurn},
		input{name: "fold without reach", verts: foldVerts, tris: foldTris, delta: foldTurn},
	)
	for _, in := range inputs {
		t.Run(in.name, func(t *testing.T) {
			ctx := t.Context()
			if in.reach > 0 {
				ctx = gaptol.With(ctx, in.reach)
			}
			beta := make([]float64, len(in.verts))
			for i := range beta {
				beta[i] = float64(i%7) * 1e-12
			}
			workers := orderedwork.EachWorkers(len(in.verts), placeParallelMin)
			// The second charge set has no source slack and a large stretch, so
			// the source area's last bits reach the area slack.
			for _, charges := range [][4]float64{{1e-6, 2e-6, 3e-16, 5e-16}, {0, 0, 1e-3, 2e-3}} {
				for _, gap := range []*denotingGap{nil, {linear: 1e-15, translation: 2e-15}} {
					volSymDiff, areaSlack, stretch, volumeCharge := charges[0], charges[1], charges[2], charges[3]
					if gap != nil {
						stretch, volumeCharge = 0, 0
					}
					want, wantErr := serialPlaceMeshGap(ctx, proofbound.NewWorkBudget(ctx), in.verts,
						in.tris, beta, volSymDiff, areaSlack, stretch, volumeCharge, in.delta, workers, gap)
					got, gotErr := placeMeshGap(ctx, proofbound.NewWorkBudget(ctx), in.verts, in.tris,
						beta, volSymDiff, areaSlack, stretch, volumeCharge, in.delta, workers, gap)
					if wantErr != nil {
						require.Error(t, gotErr)
						require.Equal(t, wantErr.Error(), gotErr.Error())
						continue
					}
					require.NoError(t, gotErr)
					requireSamePlacement(t, want, got)
				}
			}
		})
	}
	// The fold cases must cover a refusal and an accepted repair.
	_, err = PlaceMesh(t.Context(), proofbound.NewWorkBudget(t.Context()), foldVerts, foldTris,
		make([]float64, len(foldVerts)), 0, 0, 0, 0, foldTurn)
	require.ErrorIs(t, err, decaderr.ErrUnsupported)
	reachCtx := gaptol.With(t.Context(), 1e-3)
	placed, err := PlaceMesh(reachCtx, proofbound.NewWorkBudget(reachCtx), foldVerts, foldTris,
		make([]float64, len(foldVerts)), 0, 0, 0, 0, foldTurn)
	require.NoError(t, err)
	require.NotNil(t, placed.EmbeddingReach)
}
