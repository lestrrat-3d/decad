package prismpattern_test

import (
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/prismpattern"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/stretchr/testify/require"
)

func TestPrismPatternRootNeedsExactInwardOverlap(t *testing.T) {
	root := sectionrecord.ArcSeg{
		Center: sectionrecord.Point2{}, Start: sectionrecord.Point2{U: 3, V: -4}, End: sectionrecord.Point2{U: 3, V: 4},
	}
	radiusSq := prismpattern.RadiusSq(5)
	require.False(t, prismpattern.RootInside(root, radiusSq, big.NewRat(1, 1)))
	require.False(t, prismpattern.RootInside(root, radiusSq, big.NewRat(2, 1)))
	require.True(t, prismpattern.RootInside(root, radiusSq, big.NewRat(99, 100)))
	root.End = sectionrecord.Point2{U: 4, V: 3}
	require.False(t, prismpattern.RootInside(root, radiusSq, big.NewRat(1, 1)))
}
