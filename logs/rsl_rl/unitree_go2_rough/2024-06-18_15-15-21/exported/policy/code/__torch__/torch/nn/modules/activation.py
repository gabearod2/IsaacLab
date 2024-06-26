class ELU(Module):
  __parameters__ = []
  __buffers__ = []
  training : bool
  _is_full_backward_hook : NoneType
  alpha : Final[float] = 1.
  inplace : Final[bool] = False
  def forward(self: __torch__.torch.nn.modules.activation.ELU,
    input: Tensor) -> Tensor:
    _0 = __torch__.torch.nn.functional.elu(input, 1., False, )
    return _0
