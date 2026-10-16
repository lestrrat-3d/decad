package apitest_test

import (
	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/stretchr/testify/require"
	"testing"
)

func TestValidationOwnerCensus(t *testing.T) {
	op, check := validationConsumer(t)
	meshbool.ValidationOwnerCensusBegin()
	got, err := op(t.Context())
	calls, eligible, over, empty, maximum, hist, samples := meshbool.ValidationOwnerCensusEnd()
	require.NoError(t, err)
	check(got)
	if calls == 0 {
		t.Fatal("no adjacency calls observed")
	}
	t.Logf("CENSUS calls=%d eligible=%d overcap=%d empty=%d max_triangles=%d histogram=%v first32=%v", calls, eligible, over, empty, maximum, hist, samples)
}
