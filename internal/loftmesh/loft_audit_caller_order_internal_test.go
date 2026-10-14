package loftmesh

import (
	"errors"
	"math/rand/v2"
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

// callerOrderShortcuts are the shortcut sets every caller-order test runs: the
// exact-only path, the certificates, and the certificates with the sweep.
var callerOrderShortcuts = []LoftAuditShortcuts{
	{Certificates: true},
	{Certificates: true, Sweep: true},
	{},
}

// TestAuditLoftPairDataRefusesForeignCornerOrder builds the table from
// triangles {0,1,2} and {0,3,4}, then passes the caller's {0,1,2} and {3,0,4}.
// Read by position, the caller's second triangle makes vertex 3 the shared
// point, and the separating-plane certificate would admit a pair the exact
// path refuses. The two triangles cross each other away from vertex 0.
func TestAuditLoftPairDataRefusesForeignCornerOrder(t *testing.T) {
	t.Parallel()
	verts := []r3.Vec{{X: -3, Y: 4, Z: -3}, {X: 3, Y: -2, Z: -3}, {X: 4, Y: 3}, {X: -5, Y: 3, Z: -2}, {X: 4, Y: 2, Z: -4}}
	table := [][3]int{{0, 1, 2}, {0, 3, 4}}
	caller := [][3]int{{0, 1, 2}, {3, 0, 4}}
	data := NewLoftAuditData(verts, table)

	// The triangles genuinely cross: their contact is not the shared vertex.
	contact, err := meshbool.TriTriClassify(data.corners(0), data.corners(1), data.xtri(0), data.xtri(1),
		data.Norms[0], data.Norms[1])
	require.NoError(t, err)
	require.False(t, contact.Kind == meshbool.ContactPoint && contact.P0.Key() == data.Xverts[0].Key(),
		"the fixture must cross away from the shared vertex")
	_, refErr := auditLoftPairDataSignedReference(data, table, 0, 1, LoftAuditShortcuts{})
	require.ErrorIs(t, refErr, decaderr.ErrDegenerate, "the exact reference refuses the matching order")

	for _, sc := range callerOrderShortcuts {
		out, err := AuditLoftPairData(data, caller, 0, 1, sc)
		require.ErrorIs(t, err, ErrLoftAuditOrder, "shortcuts %+v returned outcome %v, err %v for a pair that crosses", sc, out, err)
		_, err = AuditLoftPairData(data, table, 0, 1, sc)
		require.ErrorIs(t, err, decaderr.ErrDegenerate, "shortcuts %+v", sc)
	}
}

// TestAuditLoftPairDataCallerOrderRandom audits 9,000 random shared-vertex
// pairs (3,000 point sets, three random caller corner orders each). A caller
// order that equals the table's keeps the exact-only reference's outcome and
// error bit for bit; any other order is refused with ErrLoftAuditOrder.
func TestAuditLoftPairDataCallerOrderRandom(t *testing.T) {
	t.Parallel()
	rng := rand.New(rand.NewPCG(1795, 7))
	table := [][3]int{{0, 1, 2}, {0, 3, 4}}
	audits, refused, matched, wrong, differ := 0, 0, 0, 0, 0
	for audits < 9000 {
		var verts []r3.Vec
		for range 5 {
			verts = append(verts, r3.Vec{X: float64(rng.IntN(11) - 5), Y: float64(rng.IntN(11) - 5), Z: float64(rng.IntN(11) - 5)})
		}
		if TriangleCollapsed(verts, table[0]) || TriangleCollapsed(verts, table[1]) {
			continue
		}
		data := NewLoftAuditData(verts, table)
		for range 3 {
			caller := permuteTriples(rng, table)
			sc := callerOrderShortcuts[rng.IntN(len(callerOrderShortcuts))]
			audits++
			got, err := AuditLoftPairData(data, caller, 0, 1, sc)
			if caller[0] != table[0] || caller[1] != table[1] {
				refused++
				if errorsIsOrder(err) {
					continue
				}
				wrong++
				// Count the audits whose verdict also differs from the exact
				// reference run on the table's own order.
				want, wantErr := auditLoftPairDataSignedReference(data, table, 0, 1, LoftAuditShortcuts{})
				if (wantErr == nil) != (err == nil) || (err == nil && !sc.Certificates && want != got) {
					differ++
				}
				continue
			}
			matched++
			want, wantErr := auditLoftPairDataSignedReference(data, table, 0, 1, sc)
			if sc.Certificates {
				// The certificates may decide sooner but never differ in verdict.
				require.Equal(t, wantErr == nil, err == nil, "verts %v", verts)
				continue
			}
			require.Equal(t, want, got, "verts %v", verts)
			if wantErr != nil {
				require.EqualError(t, err, wantErr.Error(), "verts %v", verts)
				continue
			}
			require.NoError(t, err, "verts %v", verts)
		}
	}
	require.Zero(t, wrong, "of %d mismatched-order audits, %d were not refused with ErrLoftAuditOrder "+
		"and %d of those differ from the exact reference", refused, wrong, differ)
	require.Positive(t, refused)
	require.Positive(t, matched)
}

func errorsIsOrder(err error) bool { return errors.Is(err, ErrLoftAuditOrder) }
