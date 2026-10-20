package loftcontact_test

import (
	"errors"
	"testing"

	"github.com/lestrrat-3d/decad/internal/loftcontact"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

// mine is the producer of the bodies built here.
const mine = provenance.Producer(3)

func originsOf(producer provenance.Producer, roles ...string) []provenance.Ref {
	var out []provenance.Ref
	for _, r := range roles {
		out = append(out, provenance.Ref{Producer: producer, Role: r})
	}
	return out
}

func bodyOfFaces(faces ...*topo.Face) *topo.Body {
	return &topo.Body{Origin: provenance.Ref{Producer: mine},
		Lumps: []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: faces}}}}}
}

func unlimited() *proofbound.WorkBudget {
	return &proofbound.WorkBudget{StepFn: func() error { return nil }, ErrFn: func() error { return nil }}
}

func TestSourceFaces(t *testing.T) {
	t.Parallel()

	const theirs = provenance.Producer(4)
	a := &topo.Face{Origins: originsOf(mine, "capStart", "side(0)")}
	b := &topo.Face{Origins: append(originsOf(mine, "capEnd"), originsOf(theirs, "capStart")...)}
	c := &topo.Face{Origins: originsOf(theirs, "side(1)")}

	faces, ok, err := loftcontact.SourceFaces(bodyOfFaces(a, b, c), unlimited())
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, map[string]*topo.Face{"capStart": a, "side(0)": a, "capEnd": b}, faces,
		"only the body's own producer's roles are mapped")

	twice := &topo.Face{Origins: originsOf(mine, "capStart", "capStart")}
	faces, ok, err = loftcontact.SourceFaces(bodyOfFaces(twice), unlimited())
	require.NoError(t, err)
	require.True(t, ok, "one role stamped twice on one face is fine")
	require.Equal(t, map[string]*topo.Face{"capStart": twice}, faces)

	clash := &topo.Face{Origins: originsOf(mine, "capStart")}
	faces, ok, err = loftcontact.SourceFaces(bodyOfFaces(a, clash), unlimited())
	require.NoError(t, err)
	require.False(t, ok, "one role on two faces is ambiguous")
	require.Nil(t, faces)

	faces, ok, err = loftcontact.SourceFaces(bodyOfFaces(), unlimited())
	require.NoError(t, err)
	require.True(t, ok)
	require.Empty(t, faces)

	faces, ok, err = loftcontact.SourceFaces(&topo.Body{Origin: provenance.Ref{Producer: mine}}, unlimited())
	require.NoError(t, err)
	require.True(t, ok)
	require.Empty(t, faces)
}

func TestSourceFacesOriginLimit(t *testing.T) {
	t.Parallel()

	over := make([]provenance.Ref, loftcontact.VolumeOutputSteps+1)
	_, ok, err := loftcontact.SourceFaces(bodyOfFaces(&topo.Face{Origins: over}), unlimited())
	require.NoError(t, err)
	require.False(t, ok, "one face over the limit")

	atLimit := make([]provenance.Ref, loftcontact.VolumeOutputSteps)
	for i := range atLimit {
		atLimit[i] = provenance.Ref{Producer: mine, Role: "r"}
	}
	_, ok, err = loftcontact.SourceFaces(bodyOfFaces(&topo.Face{Origins: atLimit}), unlimited())
	require.NoError(t, err)
	require.True(t, ok, "exactly the limit is allowed")

	small := make([]provenance.Ref, 100)
	big := make([]provenance.Ref, loftcontact.VolumeOutputSteps-150)
	_, ok, err = loftcontact.SourceFaces(bodyOfFaces(&topo.Face{Origins: small}, &topo.Face{Origins: small},
		&topo.Face{Origins: big}), unlimited())
	require.NoError(t, err)
	require.False(t, ok, "three faces together over the limit")
	_, ok, err = loftcontact.SourceFaces(bodyOfFaces(&topo.Face{Origins: small}, &topo.Face{Origins: small},
		&topo.Face{Origins: big[:len(big)-1]}), unlimited())
	require.NoError(t, err)
	require.False(t, ok, "three faces one under the over-limit total: 200 + limit-151 = limit+49 is still over")

	half := make([]provenance.Ref, loftcontact.VolumeOutputSteps/2+1)
	_, ok, err = loftcontact.SourceFaces(bodyOfFaces(&topo.Face{Origins: half}, &topo.Face{Origins: half}), unlimited())
	require.NoError(t, err)
	require.False(t, ok, "two faces together over the limit")

	_, ok, err = loftcontact.SourceFaces(bodyOfFaces(&topo.Face{Origins: half}, &topo.Face{Origins: half[:len(half)-2]}), unlimited())
	require.NoError(t, err)
	require.True(t, ok, "two faces exactly at the limit")
}

func TestSourceFacesChargesTheBudget(t *testing.T) {
	t.Parallel()

	face := &topo.Face{Origins: originsOf(mine, "capStart")}
	body := bodyOfFaces(face)

	// Count the steps a full run takes, then refuse each one in turn.
	var steps int
	counting := &proofbound.WorkBudget{StepFn: func() error { steps++; return nil }, ErrFn: func() error { return nil }}
	_, ok, err := loftcontact.SourceFaces(body, counting)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, 5+1+2*1, steps, "one lump, one shell, one face and one origin")
	single := steps

	steps = 0
	crowded := bodyOfFaces(
		&topo.Face{Origins: originsOf(mine, "a", "b")},
		&topo.Face{Origins: originsOf(mine, "c")},
		&topo.Face{Origins: originsOf(4, "d")})
	_, ok, err = loftcontact.SourceFaces(crowded, counting)
	require.NoError(t, err)
	require.True(t, ok)
	require.Equal(t, 5+3+2*4, steps, "every origin is charged twice and every face once")

	for refuseAt := 1; refuseAt <= single; refuseAt++ {
		seen := 0
		stop := errors.New("budget spent")
		budget := &proofbound.WorkBudget{
			StepFn: func() error {
				seen++
				if seen == refuseAt {
					return stop
				}
				return nil
			},
			ErrFn: func() error { return nil },
		}
		_, _, err := loftcontact.SourceFaces(body, budget)
		require.ErrorIs(t, err, stop, "step %d of %d", refuseAt, single)
	}

	stop := errors.New("context done")
	_, _, err = loftcontact.SourceFaces(body, &proofbound.WorkBudget{
		StepFn: func() error { return nil }, ErrFn: func() error { return stop }})
	require.ErrorIs(t, err, stop)
}
