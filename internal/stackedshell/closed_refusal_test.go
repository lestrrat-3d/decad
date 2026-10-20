package stackedshell_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/breprecord"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/momentinput"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/stackedrecord"
	"github.com/lestrrat-3d/decad/internal/stackedshell"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

func TestRefuseClosed(t *testing.T) {
	t.Parallel()

	holed := momentinput.Profile{Holes: []sectionrecord.LoopRecord{{}}}
	tests := []struct {
		name    string
		payload topo.Payload
		want    string
	}{
		{"faceted boolean result", facetedrecord.Payload{}, "modify-reach SX9"},
		{"brep receiver", breprecord.Payload{}, "modify-general SG2"},
		{"stacked receiver", stackedrecord.Payload{}, "modify-general SG2"},
		{"holed prism section", prismrecord.Payload{}, "holed prism section"},
		{"revolve with a meridian hole", revolveplan.Payload{Profile: holed}, "meridian holds a hole"},
		{"partial revolve", revolveplan.Payload{}, "partial revolve keeps both angular caps"},
		{"other receiver", nil, "shells a straight prism only"},
	}
	for _, tc := range tests {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			err := stackedshell.RefuseClosed(tc.payload)
			require.ErrorIs(t, err, decaderr.ErrUnsupported)
			require.ErrorContains(t, err, tc.want)
		})
	}
}
