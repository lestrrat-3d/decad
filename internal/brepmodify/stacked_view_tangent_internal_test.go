package brepmodify

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// tangentRecord is a record of three swept walls K, G and B around the point
// V = (0, 1): K and B run on the unit circle about the origin through V, and G
// leaves V on the circle about (shift, 2), so G turns from K by sin θ ≈ shift.
// Face i of the record is wall(i): K is 0, G is 1, B is 2.
type tangentRecord struct {
	body     *topo.Body
	bp       breprecord.Payload
	b, g, k  *topo.Face
	producer provenance.Producer
}

func newTangentRecord(t *testing.T, shift float64) tangentRecord {
	t.Helper()
	p := func(u, v float64) sectionrecord.Point2 { return sectionrecord.Point2{U: u, V: v} }
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	k := sectionrecord.ArcSeg{Center: p(0, 0), Start: p(1, 0), End: p(0, 1), TStart: 0, TEnd: 1}
	g := sectionrecord.ArcSeg{Center: p(shift, 2), Start: p(-1, 2), End: p(0, 1), TStart: 1, TEnd: 0}
	b := sectionrecord.ArcSeg{Center: p(0, 0), Start: p(0, 1), End: p(-1, 0), TStart: 0, TEnd: 1}
	bp := breprecord.Payload{
		Faces: []breprecord.Face{
			{Frame: frame, Wall: k, Z0: 0, Z1: 6},
			{Frame: frame, Wall: g, Z0: 0, Z1: 5},
			{Frame: frame, Wall: b, Z0: 5, Z1: 6},
		},
		Xform: r3.Identity(),
	}
	bp.AssignRoles()
	const producer = provenance.Producer(7)
	body := &topo.Body{Origin: provenance.Ref{Producer: producer}}
	view := func(i int) *topo.Face {
		return &topo.Face{Origins: []provenance.Ref{{Producer: producer, Role: bp.Faces[i].Role}}}
	}
	return tangentRecord{body: body, bp: bp, k: view(0), g: view(1), b: view(2), producer: producer}
}

func (r tangentRecord) prove() (bool, float64) {
	proof, ok := stackedTangentEnds(r.body, r.bp).Prove(r.b, r.g, r.k)
	return ok, proof.Turn
}

func TestRecordTangentEnds(t *testing.T) {
	t.Run("an exactly tangent corner proves with no turn", func(t *testing.T) {
		r := newTangentRecord(t, 0)
		proof, ok := stackedTangentEnds(r.body, r.bp).Prove(r.b, r.g, r.k)
		require.True(t, ok)
		require.Zero(t, proof.Turn)
		require.Equal(t, r3.NewVec(0, 1, 5), proof.At)
	})
	t.Run("a near tangent corner proves with a tiny upper turn", func(t *testing.T) {
		const shift = 1.0 / (1 << 40)
		r := newTangentRecord(t, shift)
		ok, turn := r.prove()
		require.True(t, ok)
		require.Greater(t, turn, shift)
		require.Less(t, turn, 2*shift)
	})
	t.Run("a kinked corner refuses", func(t *testing.T) {
		r := newTangentRecord(t, 0.5)
		ok, _ := r.prove()
		require.False(t, ok)
	})
	t.Run("a cusp between B and G alone refuses", func(t *testing.T) {
		r := newTangentRecord(t, 0)
		// G leaves B's end instead of K's: K and G share no endpoint.
		r.bp.Faces[1].Wall = sectionrecord.ArcSeg{
			Center: sectionrecord.Point2{U: -2, V: 0}, Start: sectionrecord.Point2{U: -2, V: 1},
			End: sectionrecord.Point2{U: -1, V: 0}, TStart: 1, TEnd: 0,
		}
		ok, _ := r.prove()
		require.False(t, ok)
	})
	t.Run("mismatched levels refuse", func(t *testing.T) {
		for name, edit := range map[string]func(*tangentRecord){
			"G ends below B starts": func(r *tangentRecord) { r.bp.Faces[1].Z1 = 4.5 },
			"K starts at A":         func(r *tangentRecord) { r.bp.Faces[0].Z0 = 5 },
			"K ends at A":           func(r *tangentRecord) { r.bp.Faces[0].Z1 = 5 },
		} {
			t.Run(name, func(t *testing.T) {
				r := newTangentRecord(t, 0)
				edit(&r)
				ok, _ := r.prove()
				require.False(t, ok)
			})
		}
	})
	t.Run("different frames refuse", func(t *testing.T) {
		r := newTangentRecord(t, 0)
		other, err := r3.NewFrame(r3.NewVec(1, 0, 0), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
		require.NoError(t, err)
		r.bp.Faces[2].Frame = other
		ok, _ := r.prove()
		require.False(t, ok)
	})
	t.Run("a face of another producer refuses", func(t *testing.T) {
		r := newTangentRecord(t, 0)
		r.k.Origins[0].Producer = r.producer + 1
		ok, _ := r.prove()
		require.False(t, ok)
	})
	t.Run("a role two record faces share refuses", func(t *testing.T) {
		r := newTangentRecord(t, 0)
		r.bp.Faces[2].Role = r.bp.Faces[0].Role
		ok, _ := r.prove()
		require.False(t, ok)
	})
	t.Run("a K that does not meet B at the shared point refuses", func(t *testing.T) {
		r := newTangentRecord(t, 0)
		r.bp.Faces[2].Wall = sectionrecord.ArcSeg{
			Center: sectionrecord.Point2{U: 0, V: 0}, Start: sectionrecord.Point2{U: 0, V: -1},
			End: sectionrecord.Point2{U: -1, V: 0}, TStart: 0, TEnd: 1,
		}
		ok, _ := r.prove()
		require.False(t, ok)
	})
}
