package stepbrep_test

import (
	"testing"

	"github.com/lestrrat-3d/decad/internal/stepbrep"
	"github.com/lestrrat-3d/decad/internal/stepread"
	"github.com/stretchr/testify/require"
)

func TestReadRefusesAModelWithNoSolid(t *testing.T) {
	t.Parallel()
	m, err := stepread.Parse([]byte("DATA;#1=CARTESIAN_POINT('',(0.,0.,0.));ENDSEC;"))
	require.NoError(t, err)
	_, err = stepbrep.Read(m, 100)
	require.ErrorIs(t, err, stepbrep.ErrMalformed)
}

func TestReadRefusesAShellThatNamesNothing(t *testing.T) {
	t.Parallel()
	m, err := stepread.Parse([]byte("DATA;#1=MANIFOLD_SOLID_BREP('',#2);#2=CLOSED_SHELL('',(#3));ENDSEC;"))
	require.NoError(t, err)
	_, err = stepbrep.Read(m, 100)
	require.ErrorIs(t, err, stepbrep.ErrMalformed, "face #3 is not defined")
}
