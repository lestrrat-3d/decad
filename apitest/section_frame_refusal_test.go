package apitest_test

import (
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

type sectionFrameCase struct {
	name   string
	origin r3.Vec
	u, v   r3.Vec
}

func requireSectionFramesRefused(t *testing.T, source *decad.Body, cases []sectionFrameCase) {
	t.Helper()
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			frame, err := r3.NewFrame(tc.origin, tc.u, tc.v)
			require.NoError(t, err)
			section, profiles, err := source.Section(t.Context(), frame)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.Nil(t, section)
			require.Nil(t, profiles)
		})
	}
}
