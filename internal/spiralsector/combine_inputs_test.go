package spiralsector_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/spiralsector"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

// The tool-count gate and the first-join certificate both refuse before any
// copy is read; the copy-lineage and rotation gates are exercised by the root
// package's spiral Union tests on real tooth bodies.
func TestCombineInputsGates(t *testing.T) {
	t.Parallel()

	tool := func() *topo.Body { return &topo.Body{} }
	many := make([]*topo.Body, 32)
	for i := range many {
		many[i] = tool()
	}
	for name, tools := range map[string][]*topo.Body{
		"no tools": nil, "one tool": {tool()}, "too many tools": many,
	} {
		_, err := spiralsector.CombineInputs(t.Context(), &topo.Body{}, tools)
		require.ErrorIs(t, err, decaderr.ErrUnsupported, name)
		require.ErrorContains(t, err, "two to 31 tools", name)
	}
	for name, count := range map[string]int{"two tools": 2, "31 tools": 31} {
		tools := many[:count]
		_, err := spiralsector.CombineInputs(t.Context(), &topo.Body{}, tools)
		require.ErrorIs(t, err, decaderr.ErrUnsupported, name)
		require.ErrorContains(t, err, "no spiral source certificate", name)
	}
}
