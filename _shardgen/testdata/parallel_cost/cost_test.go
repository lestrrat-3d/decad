package parallel_cost

import (
	"testing"
	"time"
)

func TestCostHierarchy(t *testing.T) {
	t.Run("direct/parallel", func(t *testing.T) {
		t.Parallel()
		t.Run("sequential", func(t *testing.T) {
			time.Sleep(20 * time.Millisecond)
		})
		t.Run("parallel/grandchild", func(t *testing.T) {
			t.Parallel()
			time.Sleep(30 * time.Millisecond)
		})
	})
	t.Run("sequentialParent", func(t *testing.T) {
		t.Run("parallel/grandchild", func(t *testing.T) {
			t.Parallel()
			time.Sleep(20 * time.Millisecond)
		})
	})
	t.Run("sequentialChild", func(t *testing.T) {
		time.Sleep(20 * time.Millisecond)
	})
}
