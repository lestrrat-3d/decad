// Package selectorquery evaluates the recorded clauses and cardinality of
// decad's edge and face selectors. The root package supplies live topology.
package selectorquery

import (
	"fmt"
	"strings"

	"github.com/lestrrat-3d/decad/internal/decaderr"
)

type CardKind int

const (
	CardNone CardKind = iota
	CardExactly
	CardAtLeast
)

type Cardinality struct {
	Kind CardKind
	N    int
}

// Enforce checks the count after all branches have been combined.
func (c Cardinality) Enforce(n int, what string) error {
	if c.Kind != CardNone && c.N <= 0 {
		return fmt.Errorf(`%w: a cardinality assertion needs a positive count, got %d`, decaderr.ErrDegenerate, c.N)
	}
	switch c.Kind {
	case CardNone:
		if n == 0 {
			return fmt.Errorf(`%w: the query matched no %s`, decaderr.ErrNoMatch, what)
		}
	case CardExactly:
		if n != c.N {
			return fmt.Errorf(`%w: the query matched %d %s, asserted exactly %d`, decaderr.ErrCardinality, n, what, c.N)
		}
	case CardAtLeast:
		if n < c.N {
			return fmt.Errorf(`%w: the query matched %d %s, asserted at least %d`, decaderr.ErrCardinality, n, what, c.N)
		}
	default:
		return fmt.Errorf(`%w: unknown cardinality kind %d`, decaderr.ErrDegenerate, int(c.Kind))
	}
	return nil
}

func (c Cardinality) Expected() string {
	switch c.Kind {
	case CardExactly:
		return fmt.Sprintf("exactly %d", c.N)
	case CardAtLeast:
		return fmt.Sprintf("at least %d", c.N)
	default:
		return "any"
	}
}

func (c Cardinality) Suffix() string {
	switch c.Kind {
	case CardExactly:
		return fmt.Sprintf(".exactly(%d)", c.N)
	case CardAtLeast:
		return fmt.Sprintf(".at_least(%d)", c.N)
	default:
		return ""
	}
}

// Render assembles the ordered union branches and trailing cardinality token.
func Render(kind string, branches [][]string, card Cardinality) string {
	var b strings.Builder
	b.WriteString(kind)
	for i, preds := range branches {
		if i > 0 {
			b.WriteString(".or")
		}
		b.WriteByte('(')
		b.WriteString(strings.Join(preds, ", "))
		b.WriteByte(')')
	}
	if len(branches) == 0 {
		b.WriteString("()")
	}
	b.WriteString(card.Suffix())
	return b.String()
}

// MatchAny combines conjunctions by union. An absent branch list matches all.
func MatchAny[T any, P any](item T, branches [][]P, matches func(P, T) bool) bool {
	if len(branches) == 0 {
		return true
	}
	for _, branch := range branches {
		all := true
		for _, predicate := range branch {
			if !matches(predicate, item) {
				all = false
				break
			}
		}
		if all {
			return true
		}
	}
	return false
}

type Residual struct {
	Branch    int
	Predicate string
	Remaining int
}

// Residuals records the running count for each clause when selection fails.
func Residuals[T any, P any](items []T, branches [][]P, matches func(P, T) bool,
	render func(P) string) []Residual {
	var out []Residual
	for bi, branch := range branches {
		candidates := items
		for _, predicate := range branch {
			kept := make([]T, 0, len(candidates))
			for _, item := range candidates {
				if matches(predicate, item) {
					kept = append(kept, item)
				}
			}
			candidates = kept
			out = append(out, Residual{Branch: bi, Predicate: render(predicate), Remaining: len(candidates)})
		}
	}
	return out
}
