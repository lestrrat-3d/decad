package revolveshell_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/coneaxis"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func xTurn(t *testing.T, radians float64) r3.Transform {
	t.Helper()
	turn, err := r3.Rotation(r3.Vec{X: 1}, units.Radians(radians))
	require.NoError(t, err)
	return turn
}

// joinedPair is a retained serial join and the placed sector copy that would
// extend it, with every field NextSectorUnion reads set to a passing value
// and with sources far enough apart to be separate. The certificate holds no
// blank, so the build refuses as incomplete: that refusal marks a call that
// cleared every gate.
func joinedPair(t *testing.T) (joined, placed *topo.Body) {
	t.Helper()
	return joinedPairAbout(t, coneaxis.X(), xTurn(t, math.Pi/2), r3.Vec{X: 10}, r3.Vec{Y: 10})
}

// joinedPairAbout is joinedPair with a certificate about axis, its first
// sector holding the one vertex first and the copy at motion holding placed.
// The certificate carries no General reading, so GeneralCertificate passes it
// and the copy's placement gate is the one under test.
func joinedPairAbout(t *testing.T, axis coneaxis.Axis, motion r3.Transform,
	first, placedVertex r3.Vec) (joined, placed *topo.Body) {
	t.Helper()
	trim := new(facetedrecord.PointConeTrimRecord)
	record := &facetedrecord.RevolveLoftJoinRecord{
		Certificate: &facetedrecord.RevolveLoftSectorCertificate{Trim: trim,
			Section: facetedrecord.RevolveSectionCertificate{Axis: axis}},
		First: facetedrecord.Payload{Verts: []r3.Vec{first}, MeshBound: 0.1},
	}
	joined = &topo.Body{Payload: facetedrecord.Payload{RevolveLoftJoin: record, Xform: r3.Identity()}}
	placed = &topo.Body{Payload: facetedrecord.Payload{
		RevolveLoftSector: &facetedrecord.RevolveLoftSectorPlacement{Source: trim, Motion: motion},
		Xform:             motion,
		Verts:             []r3.Vec{placedVertex},
		MeshBound:         0.1,
	}}
	return joined, placed
}

func TestNextSectorUnionPlacementAboutAxis(t *testing.T) {
	t.Parallel()
	for name, axisVec := range generalAxes {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			axis := axisValue(t, axisVec)
			// first lies off the axis, so a half turn moves it 20 mm away.
			first := r3.Vec{Z: 10}
			union := func(motion r3.Transform) (bool, error) {
				joined, placed := joinedPairAbout(t, axis, motion, first, motion.Apply(first))
				_, ok, err := revolveshell.NextSectorUnion(t.Context(), meshbool.OpUnion, topo.Env{}, joined, placed)
				return ok, err
			}

			// r3.Rotation about the float-normalized axis drifts by rounding
			// only, so the copy reaches the build, which refuses the
			// certificate's missing blank.
			ok, err := union(turnAbout(t, axisVec, math.Pi))
			require.True(t, ok, "a copy that only drifts by rounding")
			require.ErrorContains(t, err, "sector source is incomplete or stale")

			declines := map[string]r3.Transform{
				"tilted past DriftSinLimit": tiltedTurnAbout(t, axisVec, math.Pi, 0x1p-40, first),
				"reverses the axis":         turnAbout(t, r3.Vec{Z: 1}, math.Pi),
			}
			shift, err := r3.Translation(r3.Vec{Z: 1})
			require.NoError(t, err)
			declines["translated"], err = turnAbout(t, axisVec, math.Pi).Then(shift)
			require.NoError(t, err)
			mirror, err := r3.NewFrame(r3.Vec{}, unitOf(axisVec), r3.Vec{Z: 1})
			require.NoError(t, err)
			declines["reflected"], err = r3.Reflection(mirror)
			require.NoError(t, err)
			for what, motion := range declines {
				ok, err := union(motion)
				require.NoError(t, err, what)
				require.False(t, ok, what)
			}
		})
	}
}

func TestNextSectorUnionDeclines(t *testing.T) {
	t.Parallel()

	edit := func(body *topo.Body, f func(*facetedrecord.Payload)) {
		p := body.Payload.(facetedrecord.Payload)
		f(&p)
		body.Payload = p
	}
	tests := map[string]func(joined, placed *topo.Body){
		"no retained join": func(j, _ *topo.Body) {
			edit(j, func(p *facetedrecord.Payload) { p.RevolveLoftJoin = nil })
		},
		"join is placed": func(j, _ *topo.Body) {
			edit(j, func(p *facetedrecord.Payload) { p.Xform = xTurn(t, 1) })
		},
		"copy has no sector record": func(_, c *topo.Body) {
			edit(c, func(p *facetedrecord.Payload) { p.RevolveLoftSector = nil })
		},
		"copy has no source": func(_, c *topo.Body) {
			edit(c, func(p *facetedrecord.Payload) { p.RevolveLoftSector.Source = nil })
		},
		"join has no certificate": func(j, _ *topo.Body) {
			edit(j, func(p *facetedrecord.Payload) { p.RevolveLoftJoin.Certificate = nil })
		},
		"certificate is for another source": func(j, _ *topo.Body) {
			edit(j, func(p *facetedrecord.Payload) {
				p.RevolveLoftJoin.Certificate = &facetedrecord.RevolveLoftSectorCertificate{
					Trim: new(facetedrecord.PointConeTrimRecord)}
			})
		},
		"copy is not at its motion": func(_, c *topo.Body) {
			edit(c, func(p *facetedrecord.Payload) { p.Xform = r3.Identity() })
		},
		"motion is not an x turn": func(_, c *topo.Body) {
			turn, err := r3.Rotation(r3.Vec{Y: 1}, units.Radians(1))
			require.NoError(t, err)
			edit(c, func(p *facetedrecord.Payload) {
				p.RevolveLoftSector.Motion, p.Xform = turn, turn
			})
		},
		"sources are not separate": func(j, _ *topo.Body) {
			edit(j, func(p *facetedrecord.Payload) { p.RevolveLoftJoin.First.Verts = nil })
		},
	}
	for name, mutate := range tests {
		for _, swapped := range []bool{false, true} {
			joined, placed := joinedPair(t)
			mutate(joined, placed)
			a, b := joined, placed
			if swapped {
				a, b = placed, joined
			}
			_, ok, err := revolveshell.NextSectorUnion(t.Context(), meshbool.OpUnion, topo.Env{}, a, b)
			require.NoError(t, err, name)
			require.False(t, ok, "%s (swapped %v)", name, swapped)
		}
	}

	joined, placed := joinedPair(t)
	_, ok, err := revolveshell.NextSectorUnion(t.Context(), meshbool.OpCut, topo.Env{}, joined, placed)
	require.NoError(t, err)
	require.False(t, ok, "only a Union rebuilds the join")

	for _, swapped := range []bool{false, true} {
		a, b := joined, placed
		if swapped {
			a, b = placed, joined
		}
		_, ok, err := revolveshell.NextSectorUnion(t.Context(), meshbool.OpUnion, topo.Env{}, a, b)
		require.True(t, ok, "a complete pair reaches the build in either order (swapped %v)", swapped)
		require.ErrorIs(t, err, decaderr.ErrUnsupported)
		require.ErrorContains(t, err, "sector source is incomplete or stale")
	}
}

func TestSectorUnionsDeclineWhatTheyCannotCertify(t *testing.T) {
	t.Parallel()

	plain := &topo.Body{Payload: facetedrecord.Payload{Xform: r3.Identity()}}
	other := &topo.Body{}
	_, ok, err := revolveshell.SectorUnion(t.Context(), meshbool.OpUnion, topo.Env{}, plain, other)
	require.NoError(t, err)
	require.False(t, ok)
	_, ok, err = revolveshell.SectorUnion(t.Context(), meshbool.OpCut, topo.Env{}, plain, other)
	require.NoError(t, err)
	require.False(t, ok)
	_, ok, err = revolveshell.SectorUnionMany(t.Context(), topo.Env{}, []*topo.Body{plain, other, plain})
	require.NoError(t, err)
	require.False(t, ok)
	_, ok, err = revolveshell.SectorUnionMany(context.Background(), topo.Env{}, []*topo.Body{plain})
	require.NoError(t, err)
	require.False(t, ok)
}
