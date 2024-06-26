class _TorchPolicyExporter(Module):
  __parameters__ = []
  __buffers__ = []
  training : bool
  _is_full_backward_hook : Optional[bool]
  is_recurrent : bool
  actor : __torch__.torch.nn.modules.container.Sequential
  normalizer : __torch__.torch.nn.modules.linear.Identity
  def forward(self: __torch__.omni.isaac.lab_tasks.utils.wrappers.rsl_rl.exporter._TorchPolicyExporter,
    x: Tensor) -> Tensor:
    actor = self.actor
    normalizer = self.normalizer
    _0 = (actor).forward((normalizer).forward(x, ), )
    return _0
  def reset(self: __torch__.omni.isaac.lab_tasks.utils.wrappers.rsl_rl.exporter._TorchPolicyExporter) -> NoneType:
    return None
