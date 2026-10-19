package ringmemo_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/ringmemo"
	"github.com/stretchr/testify/require"
)

func TestMemoKeepsFirstValueAndEvictsOldest(t *testing.T) {
	t.Parallel()
	var m ringmemo.Memo[int, string]
	_, ok := m.Load(1)
	require.False(t, ok)

	m.Store(1, "one", 2)
	m.Store(1, "uno", 2)
	value, ok := m.Load(1)
	require.True(t, ok)
	require.Equal(t, "one", value)

	m.Store(2, "two", 2)
	m.Store(3, "three", 2)
	_, ok = m.Load(1)
	require.False(t, ok)
	for key, want := range map[int]string{2: "two", 3: "three"} {
		value, ok := m.Load(key)
		require.True(t, ok)
		require.Equal(t, want, value)
	}

	m.Store(4, "four", 2)
	_, ok = m.Load(2)
	require.False(t, ok)
	value, ok = m.Load(3)
	require.True(t, ok)
	require.Equal(t, "three", value)
	value, ok = m.Load(4)
	require.True(t, ok)
	require.Equal(t, "four", value)
}
