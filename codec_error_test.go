package decad

import (
	"encoding/json"
	"testing"

	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

type codecErrorArrayEntry struct {
	Primitive bool        `json:"primitive"`
	Quantity  units.Value `json:"quantity"`
}

func TestCodecFailurePathFollowsReturnedErrorInArray(t *testing.T) {
	data := []byte(`[{"primitive":"bad","quantity":"bad"}]`)
	var entries []codecErrorArrayEntry
	err := json.Unmarshal(data, &entries)
	require.Error(t, err)

	err = codecJSONErrorAt(data, &entries, err)
	var pathErr *codecPathError
	require.ErrorAs(t, err, &pathErr)
	require.Equal(t, "[0].quantity", pathErr.path)
}
