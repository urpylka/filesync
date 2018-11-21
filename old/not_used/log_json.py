class ParamsStorage(object):
    """
    Params storage in json-file
    """
    params_dict = {}

    def __init__(self, params_file):
        self.params_file = params_file
        self.params_dict = self.read_params()

    def __getitem__(self, item):
        return self.params_dict.get(item)

    def __setitem__(self, key, value):
        self.params_dict[key] = value
        self.save_param(key, value)

    def save_param(self, name, value):
        logging.info('Save param %s=%s' % (name, value))
        params = self.read_params()
        params[name] = value
        self.write_params(params)

    def read_params(self):
        return json.load(open(self.params_file, 'r'))

    def write_params(self, params):
        json.dump(params, open(self.params_file, 'w'))


class ParamsMixin(StateMixin):
    """
    Parameters saver/reader mixin, using ``name`` field as a slug
    """
    PARAMS_FILE = config.PARAMS_FILE

    def __init__(self):
        super(ParamsMixin, self).__init__()
        self.params = ParamsStorage(self.PARAMS_FILE)

    def get_state(self):
        return self.update_state(ParamsMixin, {
            'params': self.params.params_dict
        })