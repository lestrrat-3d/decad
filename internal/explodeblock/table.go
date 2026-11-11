package explodeblock

import (
	"sync"

	"github.com/lestrrat-3d/r3"
)

// Table holds the pairwise verdicts for one direction. Entry (a, b) is the
// verdict for part a moving toward +Dir with part b standing in the way.
// Blocking between two parts does not depend on which others are present.
type Table struct {
	dir r3.Vec
	n   int
	res []Result
}

// BuildTable tests every ordered pair of items. The items must all be
// projections of the same direction. With workers above 1 the pairs are tested
// by that many goroutines; the table is the same either way.
func BuildTable(items []Instance, workers int) *Table {
	n := len(items)
	t := &Table{n: n, res: make([]Result, n*n)}
	if n > 0 {
		t.dir = items[0].P.dir
	}
	workers = max(workers, 1)
	rows := make(chan int, n)
	for a := range n {
		rows <- a
	}
	close(rows)
	var wg sync.WaitGroup
	for range workers {
		wg.Go(func() {
			for a := range rows {
				for b := range n {
					if a == b {
						continue
					}
					t.res[a*n+b] = Block(items[a], items[b])
				}
			}
		})
	}
	wg.Wait()
	return t
}

// Dir returns the direction the table was built for.
func (t *Table) Dir() r3.Vec { return t.dir }

// N returns the number of items.
func (t *Table) N() int { return t.n }

// At returns the verdict for mover a with b in the way.
func (t *Table) At(a, b int) Result { return t.res[a*t.n+b] }

// Blockers returns, in ascending order, the items that block a.
func (t *Table) Blockers(a int) []int { return t.with(a, Blocked) }

// Uncertain returns, in ascending order, the items whose verdict against a is
// undecided.
func (t *Table) Uncertain(a int) []int { return t.with(a, Undecided) }

func (t *Table) with(a int, v Verdict) []int {
	var out []int
	for b := range t.n {
		if t.res[a*t.n+b].Verdict == v {
			out = append(out, b)
		}
	}
	return out
}
