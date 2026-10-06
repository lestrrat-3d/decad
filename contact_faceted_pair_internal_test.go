package decad

import (
	"math"
	"math/rand/v2"
	"slices"
	"testing"

	"github.com/lestrrat-3d/decad/internal/proofbound"

	"github.com/lestrrat-3d/decad/internal/pair"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// placePlanarSnapshotRebuilt is placePlanarSnapshot before the snapshot
// carried its topology: every pose maps each vertex through
// exactContactTransform and runs the whole pair.CheckPlanarSolid audit.
func placePlanarSnapshotRebuilt(snapshot *planarSnapshotEntry, pose r3.Transform) (pair.PlanarSolid, bool) {
	solid := pair.PlanarSolid{Verts: make([]proofarith.DyV3, len(snapshot.solid.Verts)),
		Tris: slices.Clip(snapshot.solid.Tris), Faces: slices.Clip(snapshot.solid.Faces)}
	for i, v := range snapshot.solid.Verts {
		solid.Verts[i] = exactContactTransform(pose, v)
	}
	audited, err := pair.CheckPlanarSolid(&solid, noSweepPoll)
	if err != nil || !audited {
		return pair.PlanarSolid{}, false
	}
	return solid, true
}

// randomContactPose is a valid pose turned about a random axis through a
// random point and shifted, with offsets from 2⁻⁴⁰ to 2¹⁰ so the staged
// coordinates' exponents vary widely.
func randomContactPose(t *testing.T, rng *rand.Rand) r3.Transform {
	t.Helper()
	scale := func() float64 { return math.Ldexp(rng.Float64()*2-1, rng.IntN(51)-40) }
	axis := r3.Vec{X: rng.Float64()*2 - 1, Y: rng.Float64()*2 - 1, Z: rng.Float64()*2 - 1}
	if rng.IntN(4) == 0 {
		axis = r3.Vec{Z: 1}
	}
	turn, err := r3.RotationAround(r3.Vec{X: scale(), Y: scale(), Z: scale()}, axis,
		units.Degrees(rng.Float64()*360-180))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.Vec{X: scale(), Y: scale(), Z: scale()})
	require.NoError(t, err)
	pose, err := turn.Then(shift)
	require.NoError(t, err)
	return pose
}

func requireDyV3Equal(t *testing.T, want, got proofarith.DyV3, msg string, args ...any) {
	t.Helper()
	for axis := range 3 {
		require.Zero(t, proofarith.DyCmp(want[axis], got[axis]), append([]any{msg}, args...)...)
	}
}

// TestExactContactMapMatchesTransform holds exactContactMap, which lifts a
// pose once for many points, to exactContactTransform on random poses and
// points: zero, integer, fine and large coordinates.
//
// Leg shown to fail: apply adding ey·p[0], every non-axis pose differs.
func TestExactContactMapMatchesTransform(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(83, 89))
	coordinate := func() float64 {
		switch rng.IntN(4) {
		case 0:
			return 0
		case 1:
			return float64(rng.IntN(41) - 20)
		}
		return math.Ldexp(rng.Float64()*2-1, rng.IntN(61)-40)
	}
	for range 500 {
		pose := randomContactPose(t, rng)
		place := newExactContactMap(pose)
		for range 8 {
			p := proofarith.DyVec(r3.Vec{X: coordinate(), Y: coordinate(), Z: coordinate()})
			requireDyV3Equal(t, exactContactTransform(pose, p), place.apply(p), "pose %v point %v", pose, p)
		}
	}
}

// TestPlacePlanarSnapshotMatchesFullAudit holds placePlanarSnapshot, which
// audits only a pose's coordinate half against the snapshot's cached
// topology, to the full per-pose audit it replaces, for an exact prism and a
// held chamfered block at random poses and the identity: the same vertices,
// the same acceptance, and a classification that reads the attached data
// identically to one that rebuilds it.
//
// Leg shown to fail: placePlanarSnapshot mapping through the pose's inverse,
// the vertices differ on the first turned pose.
func TestPlacePlanarSnapshotMatchesFullAudit(t *testing.T) {
	t.Parallel()
	doc := New()
	bodies := map[string]*Body{
		"prism": internalBoxBody(t, doc, -6, -4, 6, 4, 10),
		"held":  bandChamferedBlock(t, doc),
	}
	rng := rand.New(rand.NewPCG(97, 101))
	budget := proofbound.NewWorkBudget(t.Context())
	for name, body := range bodies {
		snapshot, err := planarSnapshotOf(t.Context(), budget, body, 0)
		require.NoError(t, err, name)
		require.True(t, snapshot.ok, "%s: premise: an admitted snapshot", name)
		require.NotNil(t, snapshot.topology, name)
		var other pair.PlanarSolid
		for trial := range 60 {
			pose := r3.Identity()
			if trial > 0 {
				pose = randomContactPose(t, rng)
			}
			want, wantOK := placePlanarSnapshotRebuilt(snapshot, pose)
			got, ok, err := placePlanarSnapshot(budget, snapshot, pose)
			require.NoError(t, err, name)
			require.Equal(t, wantOK, ok, "%s trial %d", name, trial)
			require.True(t, ok, "%s trial %d: premise: an admitted pose", name, trial)
			require.Len(t, got.Verts, len(want.Verts))
			for i := range want.Verts {
				requireDyV3Equal(t, want.Verts[i], got.Verts[i], "%s trial %d vertex %d", name, trial, i)
			}
			require.Equal(t, want.Tris, got.Tris)
			require.Equal(t, want.Faces, got.Faces)
			if trial > 0 {
				wantResult, err := pair.ClassifyPlanar(&want, &other, noSweepPoll)
				require.NoError(t, err)
				gotResult, err := pair.ClassifyPlanar(&got, &other, noSweepPoll)
				require.NoError(t, err)
				require.Equal(t, wantResult.Relation, gotResult.Relation, "%s trial %d", name, trial)
				require.Equal(t, wantResult.Reason, gotResult.Reason, "%s trial %d", name, trial)
				require.Equal(t, wantResult.Gap, gotResult.Gap, "%s trial %d", name, trial)
				require.Equal(t, wantResult.Contacts, gotResult.Contacts, "%s trial %d", name, trial)
				require.Equal(t, wantResult.Crossings, gotResult.Crossings, "%s trial %d", name, trial)
			}
			other = got
		}
	}
}
