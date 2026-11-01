package topo_test

import (
	"os/exec"
	"reflect"
	"strings"
	"testing"

	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/stretchr/testify/require"
)

// floor lists the internal packages internal/topo may depend on, directly or
// through another package. internal/topo sits directly above this set and
// below every other internal package (agent-docs/evaluator-design.md §3), so no
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
	"opdiagnostic":   {},
	"surfacenormal":  {},
	"surfacegeom":    {},
	"provenance":     {},
	"prooflineage":   {},
	"radiusmemo":     {},
	"seal":           {},
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

// TestBodyMovedFields holds the Body node's field list fixed. The root Body
// handle keeps pairReports because its type names root report types.
// SweepRadii uses the leaf radiusmemo type and belongs in the node.
// Adding a field here is a deliberate change to that boundary.
func TestBodyMovedFields(t *testing.T) {
	// reflect is confined to this test: it lists the struct's field names, and
	// no static construct can enumerate them.
	typ := reflect.TypeFor[topo.Body]()
	var got []string
	for f := range typ.Fields() {
		got = append(got, f.Name)
	}
	require.Equal(t, []string{"prismConstructionState", "Solid", "PlanarConvexity", "PlanarSnapshot", "PlanarHints", "Tess", "SweepRadii", "Origin", "Volume", "VolumeLineage", "OperationWarnings", "PrivateShift", "Area", "Centroid", "Bounds", "Kind", "Doc", "Lumps", "Owner", "Payload"}, got)
}
