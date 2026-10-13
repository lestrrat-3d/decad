package shellsurvey_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/shellsurvey"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

func TestRequireSectionCavity(t *testing.T) {
	const tol = 1e-9
	thickness := func(mm float64) units.Value { return units.Millimeters(mm) }

	require.NoError(t, shellsurvey.RequireSectionCavity(thickness(1.5), 1.5, 2, false, tol))

	err := shellsurvey.RequireSectionCavity(thickness(2), 2, 2, false, tol)
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	require.Contains(t, err.Error(), "meets or exceeds the accepted maximum")

	require.NoError(t, shellsurvey.RequireSectionCavity(thickness(2), 2, 2, true, tol))

	err = shellsurvey.RequireSectionCavity(thickness(0.5), 0.5, 1e-12, false, tol)
	require.ErrorIs(t, err, decaderr.ErrDegenerate)
	require.Contains(t, err.Error(), "at or below the evaluator's rounding tolerance")
}
