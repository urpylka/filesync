class ParamsMixin(StateMixin):
    """
    Parameters saver/reader mixin, using ``name`` field as a slug
    """
    PARAMS_FILE = config.PARAMS_FILE

    def __init__(self):
        super(ParamsMixin, self).__init__()
        self.params = ParamsStorage(self.PARAMS_FILE)

    def get_state(self):
        return self.update_state(ParamsMixin, {'params': self.params.params_dict})
