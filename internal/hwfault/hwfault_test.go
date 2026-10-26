package hwfault

import (
	"testing"

	"github.com/stretchr/testify/require"
)

// recorder counts the skips it is asked for.
type recorder struct{ skips int }

func (r *recorder) Helper()     {}
func (r *recorder) Skip(...any) { r.skips++ }

func TestSkipOnlyOnWindows(t *testing.T) {
	for goos, want := range map[string]int{"windows": 1, "linux": 0, "darwin": 0} {
		var r recorder
		skip(goos, &r)
		require.Equal(t, want, r.skips, goos)
	}
}
