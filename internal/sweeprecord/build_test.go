package sweeprecord_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/chainrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sweepinput"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

// roleBody is a bare body of one shell whose faces carry one origin each,
// with the given roles, and whose payload is p.
func roleBody(p topo.Payload, roles ...string) *topo.Body {
	body := &topo.Body{Payload: p}
	shell := &topo.FaceShell{}
	for _, role := range roles {
		shell.Faces = append(shell.Faces, &topo.Face{
			Origins: []provenance.Ref{{Producer: 3, Role: role}},
			Body:    body,
		})
	}
	body.Lumps = []*topo.Lump{{Shells: []*topo.FaceShell{shell}}}
	return body
}

func roles(body *topo.Body) [][]string {
	var out [][]string
	for _, f := range body.Lumps[0].Shells[0].Faces {
		var face []string
		for _, o := range f.Origins {
			face = append(face, o.Role)
		}
		out = append(out, face)
	}
	return out
}

// TestFinishStraightPrefixesSpanZero pins Table B's path-span index on a
// one-span reduction: every side(i,j) becomes side(0,i,j), cap roles stay,
// and the body keeps the built prism inside the sweep record.
func TestFinishStraightPrefixesSpanZero(t *testing.T) {
	t.Parallel()
	built := prismrecord.Payload{Z1: 7}
	body := roleBody(built, "side(0,1)", "side(1,0)", provenance.RoleCapStart, provenance.RoleCapEnd)
	path := &sweepinput.Path{}
	sweeprecord.FinishStraight(body, sweeprecord.Payload{Path: path, SurfaceResult: true})
	require.Equal(t, [][]string{{"side(0,0,1)"}, {"side(0,1,0)"}, {provenance.RoleCapStart}, {provenance.RoleCapEnd}},
		roles(body))
	got, ok := body.Payload.(sweeprecord.Payload)
	require.True(t, ok)
	require.Equal(t, sweeprecord.Payload{Prism: built, Path: path, SurfaceResult: true}, got)
}

// TestFinishChainKeepsTheBuiltReduction pins the chain finishing step: the
// built chain prism or chain revolve replaces the record's own, and every
// wall role gains the path-span index.
func TestFinishChainKeepsTheBuiltReduction(t *testing.T) {
	t.Parallel()
	prism := chainrecord.Payload{Z1: 5}
	body := roleBody(prism, "side(0,2)")
	sweeprecord.FinishChain(body, sweeprecord.ChainPayload{})
	require.Equal(t, [][]string{{"side(0,0,2)"}}, roles(body))
	require.Equal(t, sweeprecord.ChainPayload{Chain: prism}, body.Payload)

	revolve := revolveplan.ChainPayload{Phi1: 1.5}
	body = roleBody(revolve, "side(0,0)")
	sweeprecord.FinishChain(body, sweeprecord.ChainPayload{Arc: true})
	require.Equal(t, [][]string{{"side(0,0,0)"}}, roles(body))
	require.Equal(t, sweeprecord.ChainPayload{Revolve: revolve, Arc: true}, body.Payload)
}

// TestFinishArcSwapsCapsOnlyWhenReversed pins the arc finishing step: wall
// roles gain the span index, a closed circle adds the second span's role, and
// the two cap roles swap only under ReverseArcCaps.
func TestFinishArcSwapsCapsOnlyWhenReversed(t *testing.T) {
	t.Parallel()
	for _, tc := range []struct {
		name    string
		payload sweeprecord.Payload
		want    [][]string
	}{
		{"forward", sweeprecord.Payload{Arc: true},
			[][]string{{"side(0,0,1)"}, {provenance.RoleCapStart}, {provenance.RoleCapEnd}}},
		{"reversed", sweeprecord.Payload{Arc: true, ReverseArcCaps: true},
			[][]string{{"side(0,0,1)"}, {provenance.RoleCapEnd}, {provenance.RoleCapStart}}},
		{"closed circle", sweeprecord.Payload{Arc: true, ClosedCircle: true},
			[][]string{{"side(0,0,1)", "side(1,0,1)"}, {provenance.RoleCapStart}, {provenance.RoleCapEnd}}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			built := revolveplan.Payload{Phi1: 0.5}
			body := roleBody(built, "side(0,1)", provenance.RoleCapStart, provenance.RoleCapEnd)
			sweeprecord.FinishArc(body, tc.payload)
			require.Equal(t, tc.want, roles(body))
			want := tc.payload
			want.Revolve = built
			require.Equal(t, want, body.Payload)
		})
	}
}

// TestRestoreArcCapRolesSwapsOnlyWhenReversed pins the span cap rename: the
// two cap roles swap under reverse and nothing changes otherwise.
func TestRestoreArcCapRolesSwapsOnlyWhenReversed(t *testing.T) {
	t.Parallel()
	body := roleBody(nil, "side(0,1)", provenance.RoleCapStart, provenance.RoleCapEnd)
	sweeprecord.RestoreArcCapRoles(body, false)
	require.Equal(t, [][]string{{"side(0,1)"}, {provenance.RoleCapStart}, {provenance.RoleCapEnd}}, roles(body))
	sweeprecord.RestoreArcCapRoles(body, true)
	require.Equal(t, [][]string{{"side(0,1)"}, {provenance.RoleCapEnd}, {provenance.RoleCapStart}}, roles(body))
}

// TestStraightPrismReductionAdmitsOneSolidLine pins the modifiers' reduction
// gate: one solid, one-line profile Sweep hands back its prism; an arc, a
// composite, a sheet, a closed path, a multi-record path, a missing path or
// another payload is refused.
func TestStraightPrismReductionAdmitsOneSolidLine(t *testing.T) {
	t.Parallel()
	prism := prismrecord.Payload{Z1: 9}
	line := &sweepinput.Path{Records: make([]sweepinput.PathRecord, 1), Segments: make([]sweepinput.Segment, 1)}
	arc := &sweepinput.Path{Records: make([]sweepinput.PathRecord, 1),
		Segments: []sweepinput.Segment{{Arc: true}}}
	two := &sweepinput.Path{Records: make([]sweepinput.PathRecord, 2), Segments: make([]sweepinput.Segment, 1)}
	got, ok := sweeprecord.StraightPrismReduction(sweeprecord.Payload{Prism: prism, Path: line})
	require.True(t, ok)
	require.Equal(t, prism, got)
	for name, refused := range map[string]topo.Payload{
		"arc segment":   sweeprecord.Payload{Prism: prism, Path: arc},
		"arc reduction": sweeprecord.Payload{Prism: prism, Path: line, Arc: true},
		"sheet":         sweeprecord.Payload{Prism: prism, Path: line, SurfaceResult: true},
		"closed circle": sweeprecord.Payload{Prism: prism, Path: line, ClosedCircle: true},
		"closed loop":   sweeprecord.Payload{Prism: prism, Path: line, ClosedComposite: true},
		"spans":         sweeprecord.Payload{Prism: prism, Path: line, Spans: make([]sweeprecord.SpanPayload, 1)},
		"two records":   sweeprecord.Payload{Prism: prism, Path: two},
		"no path":       sweeprecord.Payload{Prism: prism},
		"prism":         prism,
	} {
		got, ok := sweeprecord.StraightPrismReduction(refused)
		require.False(t, ok, name)
		require.Equal(t, prismrecord.Payload{}, got, name)
	}
}
