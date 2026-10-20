package tessaudit_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/patchchain"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/stitchrecord"
	"github.com/lestrrat-3d/decad/internal/sweeprecord"
	"github.com/lestrrat-3d/decad/internal/tessaudit"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

func TestAuditsFacetContact(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name    string
		payload topo.Payload
		want    bool
	}{
		{"no payload", nil, false},
		{"faceted body", facetedrecord.Payload{}, false},
		{"all-edge chamfer", breprecord.AllEdgeChamferPayload{}, true},
		{"patch edges", patchchain.Payload{}, true},
		{"body patch", patchchain.BodyPayload{}, true},
		{"revolve", revolveplan.Payload{}, true},
		{"straight sweep", sweeprecord.Payload{}, false},
		{"sweep with spans", sweeprecord.Payload{Spans: make([]sweeprecord.SpanPayload, 1)}, true},
		{"arc sweep", sweeprecord.Payload{Arc: true}, true},
		{"straight chain sweep", sweeprecord.ChainPayload{}, false},
		{"chain sweep with spans", sweeprecord.ChainPayload{Spans: make([]sweeprecord.ChainSpanPayload, 1)}, true},
		{"arc chain sweep", sweeprecord.ChainPayload{Arc: true}, true},
		{"stitch without a held triangle set", stitchrecord.Payload{}, true},
		{"stitch with a held triangle set", stitchrecord.Payload{Tris: [][3]int{{0, 1, 2}}}, false},
	}
	for _, tc := range tests {
		require.Equal(t, tc.want, tessaudit.AuditsFacetContact(tc.payload), tc.name)
	}
}
