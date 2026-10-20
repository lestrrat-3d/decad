package obliqueboss_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/obliqueboss"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// bossFixture is a vertical boss of radius 2 on a plate whose top is at z = 5,
// the one junction edge between the plate's top cap and the boss side, and
// the record that names both.
func bossFixture() (*facetedrecord.ObliqueBossRecord, []*topo.Edge) {
	plateTop := &topo.Face{Surface: surfacegeom.Plane{},
		Origins: []provenance.Ref{{Producer: 1, Role: provenance.RoleCapEnd}}}
	bossSide := &topo.Face{Surface: surfacegeom.Plane{},
		Origins: []provenance.Ref{{Producer: 2, Role: prismrecord.OuterFirstSideRole}}}
	body := func(producer provenance.Producer, face *topo.Face) *topo.Body {
		return &topo.Body{Origin: provenance.Ref{Producer: producer},
			Lumps: []*topo.Lump{{Shells: []*topo.FaceShell{{Faces: []*topo.Face{face}}}}}}
	}
	rec := &facetedrecord.ObliqueBossRecord{
		Plate: body(1, plateTop), Boss: body(2, bossSide),
		Rect: momentinput.AxisRect{U0: -20, U1: 20, V0: -20, V1: 20},
		Top:  5, Base: r3.Vec{}, Axis: r3.Vec{Z: 1}, Radius: 2, Length: 20, SourceBound: 1e-9,
	}
	v := &topo.Vertex{}
	junction := &topo.Edge{Curve: surfacegeom.FacetedCurve{}, Start: v, End: v, Faces: []*topo.Face{plateTop, bossSide}}
	return rec, []*topo.Edge{junction}
}

const (
	clearanceMsg = "exceeds its proven clearance"
	reachMsg     = "reaches a plate side or the boss far cap"
)

func TestFilletBossGates(t *testing.T) {
	t.Parallel()

	const radius = 0.25
	tests := map[string]struct {
		edit   func(rec *facetedrecord.ObliqueBossRecord, edges *[]*topo.Edge) (r, delta float64)
		msg    string
		denied string
	}{
		"no edge": {func(_ *facetedrecord.ObliqueBossRecord, e *[]*topo.Edge) (float64, float64) {
			*e = nil
			return radius, 0
		}, "needs its one complete junction edge", ""},
		"two edges": {func(_ *facetedrecord.ObliqueBossRecord, e *[]*topo.Edge) (float64, float64) {
			*e = append(*e, (*e)[0])
			return radius, 0
		}, "needs its one complete junction edge", ""},
		"open edge": {func(_ *facetedrecord.ObliqueBossRecord, e *[]*topo.Edge) (float64, float64) {
			(*e)[0].End = &topo.Vertex{}
			return radius, 0
		}, "needs its one complete junction edge", ""},
		"no faceted witness": {func(_ *facetedrecord.ObliqueBossRecord, e *[]*topo.Edge) (float64, float64) {
			(*e)[0].Curve = surfacegeom.Line3{}
			return radius, 0
		}, "no faceted edge witness", ""},
		"plate lost its top": {func(r *facetedrecord.ObliqueBossRecord, _ *[]*topo.Edge) (float64, float64) {
			r.Plate.Lumps[0].Shells[0].Faces = nil
			return radius, 0
		}, "not the complete plate-to-cylinder junction", ""},
		"boss lost its side": {func(r *facetedrecord.ObliqueBossRecord, _ *[]*topo.Edge) (float64, float64) {
			r.Boss.Lumps[0].Shells[0].Faces = nil
			return radius, 0
		}, "not the complete plate-to-cylinder junction", ""},
		"edge on one face": {func(_ *facetedrecord.ObliqueBossRecord, e *[]*topo.Edge) (float64, float64) {
			(*e)[0].Faces = (*e)[0].Faces[:1]
			return radius, 0
		}, "not the complete plate-to-cylinder junction", ""},
		"edge between other faces": {func(_ *facetedrecord.ObliqueBossRecord, e *[]*topo.Edge) (float64, float64) {
			(*e)[0].Faces = []*topo.Face{{}, {}}
			return radius, 0
		}, "not the complete plate-to-cylinder junction", ""},
		"displaced radius": {func(_ *facetedrecord.ObliqueBossRecord, _ *[]*topo.Edge) (float64, float64) {
			return radius, 1e-9
		}, clearanceMsg, ""},
		"zero radius": {func(_ *facetedrecord.ObliqueBossRecord, _ *[]*topo.Edge) (float64, float64) {
			return 0, 0
		}, clearanceMsg, ""},
		"negative radius": {func(_ *facetedrecord.ObliqueBossRecord, _ *[]*topo.Edge) (float64, float64) {
			return -1, 0
		}, clearanceMsg, ""},
		"nan radius": {func(_ *facetedrecord.ObliqueBossRecord, _ *[]*topo.Edge) (float64, float64) {
			return math.NaN(), 0
		}, clearanceMsg, ""},
		"radius past a quarter of the boss": {func(_ *facetedrecord.ObliqueBossRecord, _ *[]*topo.Edge) (float64, float64) {
			return 0.5, 0
		}, clearanceMsg, ""},
		"plate too narrow in u (low)": {func(r *facetedrecord.ObliqueBossRecord, _ *[]*topo.Edge) (float64, float64) {
			r.Rect.U0 = -1
			return radius, 0
		}, reachMsg, ""},
		"plate too narrow in u (high)": {func(r *facetedrecord.ObliqueBossRecord, _ *[]*topo.Edge) (float64, float64) {
			r.Rect.U1 = 1
			return radius, 0
		}, reachMsg, ""},
		"plate too narrow in v (low)": {func(r *facetedrecord.ObliqueBossRecord, _ *[]*topo.Edge) (float64, float64) {
			r.Rect.V0 = -1
			return radius, 0
		}, reachMsg, ""},
		"plate too narrow in v (high)": {func(r *facetedrecord.ObliqueBossRecord, _ *[]*topo.Edge) (float64, float64) {
			r.Rect.V1 = 1
			return radius, 0
		}, reachMsg, ""},
		"boss too short": {func(r *facetedrecord.ObliqueBossRecord, _ *[]*topo.Edge) (float64, float64) {
			r.Length = 5.2
			return radius, 0
		}, reachMsg, ""},
	}
	for name, tc := range tests {
		rec, edges := bossFixture()
		r, delta := tc.edit(rec, &edges)
		_, err := obliqueboss.FilletBoss(t.Context(), topo.Env{}, edges, rec, r, delta)
		require.ErrorIs(t, err, decaderr.ErrUnsupported, name)
		require.ErrorContains(t, err, tc.msg, name)
	}
}
