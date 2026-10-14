package topo_test

import (
	"os/exec"
	"strings"
	"testing"

	"github.com/stretchr/testify/require"
)

// floor lists the internal packages internal/topo may depend on, directly or
// through another package. internal/topo sits directly above this set and
// below every other internal package (docs/evaluator-design.md §3), so no
// package here may ever import internal/topo. Later node fields grow the
// closure toward this list; a package outside it fails the test.
var floor = map[string]struct{}{
	"decaderr":       {},
	"proof":          {},
	"proofbound":     {},
	"orderedwork":    {},
	"pair":           {},
	"pair/planar":    {},
	"sectionrecord":  {},
	"planarsnapshot": {},
	"denotation":     {},
	"measurement":    {},
	"surfacenormal":  {},
	"surfacegeom":    {},
	"provenance":     {},
}

func TestTopoDependsOnlyOnFloor(t *testing.T) {
	const prefix = "github.com/lestrrat-3d/decad/internal/"
	out, err := exec.CommandContext(t.Context(), "go", "list", "-deps", ".").Output()
	require.NoError(t, err)
	for pkg := range strings.FieldsSeq(string(out)) {
		if !strings.HasPrefix(pkg, prefix) {
			continue
		}
		name := strings.TrimPrefix(pkg, prefix)
		if name == "topo" {
			continue
		}
		_, ok := floor[name]
		require.True(t, ok, "internal/topo imports %s, which is outside the floor", name)
	}
}
