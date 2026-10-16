package main

import (
	"os"
	"path/filepath"
	"testing"

	"github.com/stretchr/testify/require"
)

func TestAnimatedFeatureSelection(t *testing.T) {
	renders := animatedFeatureRenders()
	names := make([]string, len(renders))
	for i, render := range renders {
		names[i] = render.name()
	}
	require.ElementsMatch(t, []string{
		"extrude", "revolve", "sweep", "loft", "fillet", "chamfer",
		"cap-chamfer", "shell", "boolean", "freeform", "surface",
	}, names)
	for _, name := range []string{"thread", "verify"} {
		_, err := selectRenders(renders, name)
		require.ErrorContains(t, err, "unknown shot")
		stills, err := selectRenders(featureRenders(), name)
		require.NoError(t, err)
		require.Len(t, stills, 1)
	}
	// Rejection happens before creating frames or invoking the renderer.
	out := filepath.Join(t.TempDir(), "not-created")
	require.ErrorContains(t, runFeatureAnimations(t.Context(), []string{"-only", "thread", "-out", out}), "unknown shot")
	_, err := os.Stat(out)
	require.ErrorIs(t, err, os.ErrNotExist)
}
