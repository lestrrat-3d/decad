package provenance_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/stretchr/testify/require"
)

func TestSourcePatchSame(t *testing.T) {
	t.Parallel()
	parent := &provenance.SourcePatch{Producer: 1}
	m0, m1 := &provenance.SourcePatch{Producer: 2}, &provenance.SourcePatch{Producer: 3}
	base := provenance.SourcePatch{Producer: 4, Operand: 1, Ordinal: 2, Parent: parent, Members: []*provenance.SourcePatch{m0, m1}}

	same := base
	same.Members = []*provenance.SourcePatch{m0, m1}
	require.True(t, base.Same(same), "equal records")
	require.True(t, provenance.SourcePatch{}.Same(provenance.SourcePatch{}), "zero records")

	cases := map[string]func(*provenance.SourcePatch){
		"producer":       func(p *provenance.SourcePatch) { p.Producer++ },
		"operand":        func(p *provenance.SourcePatch) { p.Operand++ },
		"ordinal":        func(p *provenance.SourcePatch) { p.Ordinal++ },
		"blend":          func(p *provenance.SourcePatch) { p.Blend = true },
		"parent":         func(p *provenance.SourcePatch) { p.Parent = &provenance.SourcePatch{Producer: 1} },
		"member pointer": func(p *provenance.SourcePatch) { v := *m1; p.Members = []*provenance.SourcePatch{m0, &v} },
		"member order":   func(p *provenance.SourcePatch) { p.Members = []*provenance.SourcePatch{m1, m0} },
		"member count":   func(p *provenance.SourcePatch) { p.Members = []*provenance.SourcePatch{m0} },
		"no members":     func(p *provenance.SourcePatch) { p.Members = nil },
	}
	for name, mutate := range cases {
		got := base
		got.Members = []*provenance.SourcePatch{m0, m1}
		mutate(&got)
		require.False(t, base.Same(got), name)
		require.False(t, got.Same(base), name)
	}
}
