package gfold

type Gfold struct {
	optMinMinimumFuelUse bool
}

type GfoldOption func(*Gfold)

func New(opts ...GfoldOption) *Gfold {
	const (
		defaultMinimumFuelUse = true
	)

	g := &Gfold{
		optMinMinimumFuelUse: defaultMinimumFuelUse,
	}

	for _, opt := range opts {
		opt(g)
	}

	return g
}

func WithMinimalFuelUse() GfoldOption {
	return func(h *Gfold) {
		h.optMinMinimumFuelUse = true
	}
}

func (g *Gfold) SetMinimumFuelUse(opt bool) {
	g.optMinMinimumFuelUse = opt
}
