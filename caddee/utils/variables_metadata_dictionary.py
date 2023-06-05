"""Define the OptionsDictionary class."""
from __future__ import division, print_function

from six import iteritems, string_types


# unique object to check if default is given
_undefined = object()


class VariablesMetadataDictionary(object):
    """
    Dictionary with pre-declaration of keys for value-checking and default values.

    This class is instantiated for:
        1. the options attribute in solvers, drivers, and processor allocators
        2. the supports attribute in drivers
        3. the options attribute in systems

    Attributes
    ----------
    _dict : dict of dict
        Dictionary of entries. Each entry is a dictionary consisting of value, values,
        types, desc, lower, and upper.
    _read_only : bool
        If True, no options can be set after declaration.
    """

    def __init__(self, read_only=False):
        """
        Initialize all attributes.

        Parameters
        ----------
        read_only : bool
            If True, setting (via __setitem__ or update) is not permitted.
        """
        self._dict = {}
        self._read_only = read_only

    def __repr__(self):
        """
        Return a dictionary representation of the options.

        Returns
        -------
        dict
            The options dictionary.
        """
        return self._dict.__repr__()

    def __rst__(self):
        """
        Generate reStructuredText view of the options table.

        Returns
        -------
        list of str
            A rendition of the options as an rST table.
        """
        outputs = []
        for option_name, option_data in sorted(iteritems(self._dict)):
            name = option_name
            default = option_data['value'] if option_data['value'] is not _undefined \
                else '**Required**'
            values = option_data['values']
            types = option_data['types']
            desc = option_data['desc']

            # if the default is an object instance, replace with the (unqualified) object type
            default_str = str(default)
            idx = default_str.find(' object at ')
            if idx >= 0 and default_str[0] == '<':
                parts = default_str[:idx].split('.')
                default = parts[-1]

            if types is None:
                types = "N/A"

            elif types is not None:
                if not isinstance(types, (tuple, list)):
                    types = (types,)

                types = [type_.__name__ for type_ in types]

            if values is None:
                values = "N/A"

            elif values is not None:
                if not isinstance(values, (tuple, list)):
                    values = (values,)

                values = [value for value in values]

            outputs.append([name, default, values, types, desc])

        lines = []

        col_heads = ['Option', 'Default', 'Acceptable Values', 'Acceptable Types', 'Description']

        max_sizes = {}
        for j, col in enumerate(col_heads):
            max_sizes[j] = len(col)

        for output in outputs:
            for j, item in enumerate(output):
                length = len(str(item))
                if max_sizes[j] < length:
                    max_sizes[j] = length

        header = ""
        titles = ""
        for key, val in iteritems(max_sizes):
            header += '=' * val + ' '

        for j, head in enumerate(col_heads):
            titles += "%s " % head
            size = max_sizes[j]
            space = size - len(head)
            if space > 0:
                titles += space * ' '

        lines.append(header)
        lines.append(titles)
        lines.append(header)

        n = 3
        for output in outputs:
            line = ""
            for j, item in enumerate(output):
                line += "%s " % str(item)
                size = max_sizes[j]
                space = size - len(str(item))
                if space > 0:
                    line += space * ' '

            lines.append(line)
            n += 1

        lines.append(header)

        return lines

    def __str__(self, width=100):
        """
        Generate text string representation of the options table.

        Parameters
        ----------
        width : int
            The maximum width of the text.

        Returns
        -------
        str
            A text representation of the options table.
        """
        rst = self.__rst__()
        cols = [len(header) for header in rst[0].split()]
        desc_col = sum(cols[:-1]) + len(cols) - 1
        desc_len = width - desc_col

        # if it won't fit in allowed width, just return the rST
        if desc_len < 10:
            return '\n'.join(rst)

        text = []
        for row in rst:
            if len(row) > width:
                text.append(row[:width])
                if not row.startswith('==='):
                    row = row[width:].rstrip()
                    while len(row) > 0:
                        text.append(' ' * desc_col + row[:desc_len])
                        row = row[desc_len:]
            else:
                text.append(row)

        return '\n'.join(text)

    def _assert_valid(self, name, value):
        """
        Check whether the given value is valid, where the key has already been declared.

        The optional checks consist of ensuring: the value is one of a list of acceptable values,
        the type of value is one of a list of acceptable types, value is not less than lower,
        value is not greater than upper, and value satisfies check_valid.

        Parameters
        ----------
        name : str
            The key for the declared option.
        value : object
            The default or user-set value to check for value, type, lower, and upper.
        """
        meta = self._dict[name]
        values = meta['values']
        types = meta['types']
        lower = meta['lower']
        upper = meta['upper']

        if not (value is None and meta['allow_none']):
            # If only values is declared
            if values is not None:
                if value not in values:
                    if isinstance(value, string_types):
                        value = "'{}'".format(value)
                    raise ValueError("Value ({}) of option '{}' "
                                     "is not one of {}.".format(value, name, values))
            # If only types is declared
            elif types is not None:
                if not isinstance(value, types):
                    vtype = type(value)
                    if isinstance(value, string_types):
                        value = "'{}'".format(value)
                    raise TypeError("Value ({}) of option '{}' has type of ({}), but "
                                    "expected type ({}).".format(value, name, vtype, types))

            if upper is not None:
                if value > upper:
                    raise ValueError("Value ({}) of option '{}' "
                                     "exceeds maximum allowed value of {}.".format(value, name,
                                                                                   upper))
            if lower is not None:
                if value < lower:
                    raise ValueError("Value ({}) of option '{}' "
                                     "is less than minimum allowed value of {}.".format(value, name,
                                                                                        lower))

        # General function test
        if meta['check_valid'] is not None:
            meta['check_valid'](name, value)

    def declare(self, name, default=_undefined, values=None, types=None, desc='',
                csdl_var=True, design_condition=None, units='', computed_upstream=False, 
                dv_flag=False, upper=None, lower=None, dv_scaler=None,
                check_valid=None, allow_none=False):
        r"""
        Declare a variable.

        The value of the option must satisfy the following: 
        NOTE: don't think we need these conditions for what variables_metadata
        1. If values only was given when declaring, value must be in values.
        2. If types only was given when declaring, value must satisfy isinstance(value, types).
        3. It is an error if both values and types are given.

        Parameters
        ----------
        name : str
            Name of the variable.
        default : object or Null
            Optional default value that must be valid under the above 3 conditions.
        values : set or list or tuple or None: NOTE: not needed
            Optional list of acceptable option values.
        types : type or tuple of types or None
            Optional type or list of acceptable option types.
        desc : str NOTE: not needed
            Optional description of the option.
        csdl_var : bool (True)
            Whether the variable will be csdl variable (True by default).
        units : str ('')
            Unit of the variable (if '', assume SI)
        design_condition : DesignCondition or None
            The design condition the variable corresponds to (None if not applicable).
        computed_upstream : bool (False)
            Whether the variable has been computed upstream (False by default)
            If True, the variable will be declared as csdl variable upstream and 
            won't be re-declared as such. 
        dv_flag : bool (False)
            Whether the variable will be a design variable for gradient-based optimization.
        upper : float, int, np.ndarray, or None
            Upper bound of the (design) variable during optimization.
        lower : float, int, np.ndarray, or None
            Lower bound of the (design) variable during optimization.
        dv_scaler : float, int, np.ndarray, or None
        check_valid : function or None : NOTE: is this necessary 
            General check function that raises an exception if value is not valid.
        allow_none : bool
            If True, allow None as a value regardless of values or types.
        """
        if values is not None and not isinstance(values, (set, list, tuple)):
            raise TypeError("In declaration of option '%s', the 'values' arg must be of type None,"
                            " list, or tuple - not %s." % (name, values))
        if types is not None and not isinstance(types, (type, set, list, tuple)):
            raise TypeError("In declaration of option '%s', the 'types' arg must be None, a type "
                            "or a tuple - not %s." % (name, types))

        if types is not None and values is not None:
            raise RuntimeError("'types' and 'values' were both specified for option '%s'." %
                               name)

        default_provided = default is not _undefined

        self._dict[name] = {
            'value': default,
            'values': values,
            'types': types,
            'desc': desc,
            'csdl_var': csdl_var,
            'design_condition': design_condition, 
            'units': units,
            'computed_upstream': computed_upstream,
            'dv_flag': dv_flag,
            'upper': upper,
            'lower': lower,
            'dv_scaler': dv_scaler, 
            'check_valid': check_valid,
            'has_been_set': default_provided,
            'allow_none': allow_none,
        }

        # If a default is given, check for validity
        if default_provided:
            self._assert_valid(name, default)

    def undeclare(self, name):
        """
        Remove entry from the OptionsDictionary, for classes that don't use that option.

        Parameters
        ----------
        name : str
            The name of a key, the entry of which will be removed from the internal dictionary.

        """
        if name in self._dict:
            del self._dict[name]

    def update(self, in_dict):
        """
        Update the internal dictionary with the given one.

        Parameters
        ----------
        in_dict : dict
            The incoming dictionary to add to the internal one.
        """
        for name in in_dict:
            self[name] = in_dict[name]

    def __iter__(self):
        """
        Provide an iterator.

        Returns
        -------
        iterable
            iterator over the keys in the dictionary.
        """
        return iter(self._dict)

    def __contains__(self, key):
        """
        Check if the key is in the local dictionary.

        Parameters
        ----------
        key : str
            name of the option.

        Returns
        -------
        boolean
            whether key is in the local dict.
        """
        return key in self._dict

    def __setitem__(self, name, value, csdl_var, design_condition, units,
                    computed_upstream, dv_flag, lower, upper, scaler):
        """
        Set an option in the local dictionary.

        Parameters
        ----------
        name : str
            name of the option.
        value : -
            value of the option to be value- and type-checked if declared.
        """
        try:
            meta = self._dict[name]
        except KeyError:
            # The key must have been declared.
            # msg = "Variable '{}' cannot be set because it has not been declared."
            msg = "Unknown variable '{}'. Acceptable variables are " + str(list(self._dict.keys()))
            raise KeyError(msg.format(name))

        if self._read_only:
            raise KeyError("Tried to set read-only option '{}'.".format(name))

        self._assert_valid(name, value)

        meta['value'] = value
        meta['csdl_var'] = csdl_var
        meta['design_condition'] = design_condition
        meta['units'] = units
        meta['computed_upstream'] = computed_upstream
        meta['dv_flag'] = dv_flag
        meta['lower'] = lower
        meta['upper'] = upper
        meta['scaler'] = scaler        
        meta['has_been_set'] = True

    def __getitem__(self, name):
        """
        Get an option from the dict or declared default.

        Parameters
        ----------
        name : str
            name of the option.

        Returns
        -------
        value : -
            value of the option.
        """
        # If the option has been set in this system, return the set value
        try:
            meta = self._dict[name]
            if meta['has_been_set']:
                return meta['value']
            else:
                raise RuntimeError("Option '{}' is required but has not been set.".format(name))
        except KeyError:
            raise KeyError("Option '{}' cannot be found".format(name))


# from __future__ import division, print_function

# from six import iteritems, string_types


# # unique object to check if default is given
# _undefined = object()

# class VariablesMetadataDictionary(object):
#     """
#     Dictionary with pre-declaration of keys for value-checking and default values.

#     This class is instantiated for:
#         1. the options attribute in solvers, drivers, and processor allocators
#         2. the supports attribute in drivers
#         3. the options attribute in systems

#     Attributes
#     ----------
#     _dict : dict of dict
#         Dictionary of entries. Each entry is a dictionary consisting of value, values,
#         types, desc, lower, and upper.
#     _read_only : bool
#         If True, no options can be set after declaration.
#     """

#     def __init__(self, read_only=False):
#         """
#         Initialize all attributes.

#         Parameters
#         ----------
#         read_only : bool
#             If True, setting (via __setitem__ or update) is not permitted.
#         """
#         self._dict = {}
#         self._read_only = read_only

#     def __repr__(self):
#         """
#         Return a dictionary representation of the options.

#         Returns
#         -------
#         dict
#             The options dictionary.
#         """
#         return self._dict.__repr__()

#     def __rst__(self):
#         """
#         Generate reStructuredText view of the options table.

#         Returns
#         -------
#         list of str
#             A rendition of the options as an rST table.
#         """
#         outputs = []
#         for option_name, option_data in sorted(iteritems(self._dict)):
#             name = option_name
#             default = option_data['value'] if option_data['value'] is not _undefined \
#                 else '**Required**'
#             values = option_data['values']
#             types = option_data['types']
#             desc = option_data['desc']

#             # if the default is an object instance, replace with the (unqualified) object type
#             default_str = str(default)
#             idx = default_str.find(' object at ')
#             if idx >= 0 and default_str[0] == '<':
#                 parts = default_str[:idx].split('.')
#                 default = parts[-1]

#             if types is None:
#                 types = "N/A"

#             elif types is not None:
#                 if not isinstance(types, (tuple, list)):
#                     types = (types,)

#                 types = [type_.__name__ for type_ in types]

#             if values is None:
#                 values = "N/A"

#             elif values is not None:
#                 if not isinstance(values, (tuple, list)):
#                     values = (values,)

#                 values = [value for value in values]

#             outputs.append([name, default, values, types, desc])

#         lines = []

#         col_heads = ['Option', 'Default', 'Acceptable Values', 'Acceptable Types', 'Description']

#         max_sizes = {}
#         for j, col in enumerate(col_heads):
#             max_sizes[j] = len(col)

#         for output in outputs:
#             for j, item in enumerate(output):
#                 length = len(str(item))
#                 if max_sizes[j] < length:
#                     max_sizes[j] = length

#         header = ""
#         titles = ""
#         for key, val in iteritems(max_sizes):
#             header += '=' * val + ' '

#         for j, head in enumerate(col_heads):
#             titles += "%s " % head
#             size = max_sizes[j]
#             space = size - len(head)
#             if space > 0:
#                 titles += space * ' '

#         lines.append(header)
#         lines.append(titles)
#         lines.append(header)

#         n = 3
#         for output in outputs:
#             line = ""
#             for j, item in enumerate(output):
#                 line += "%s " % str(item)
#                 size = max_sizes[j]
#                 space = size - len(str(item))
#                 if space > 0:
#                     line += space * ' '

#             lines.append(line)
#             n += 1

#         lines.append(header)

#         return lines

#     def __str__(self, width=100):
#         """
#         Generate text string representation of the options table.

#         Parameters
#         ----------
#         width : int
#             The maximum width of the text.

#         Returns
#         -------
#         str
#             A text representation of the options table.
#         """
#         rst = self.__rst__()
#         cols = [len(header) for header in rst[0].split()]
#         desc_col = sum(cols[:-1]) + len(cols) - 1
#         desc_len = width - desc_col

#         # if it won't fit in allowed width, just return the rST
#         if desc_len < 10:
#             return '\n'.join(rst)

#         text = []
#         for row in rst:
#             if len(row) > width:
#                 text.append(row[:width])
#                 if not row.startswith('==='):
#                     row = row[width:].rstrip()
#                     while len(row) > 0:
#                         text.append(' ' * desc_col + row[:desc_len])
#                         row = row[desc_len:]
#             else:
#                 text.append(row)

#         return '\n'.join(text)

#     def _assert_valid(self, name, value):
#         """
#         Check whether the given value is valid, where the key has already been declared.

#         The optional checks consist of ensuring: the value is one of a list of acceptable values,
#         the type of value is one of a list of acceptable types, value is not less than lower,
#         value is not greater than upper, and value satisfies check_valid.

#         Parameters
#         ----------
#         name : str
#             The key for the declared option.
#         value : object
#             The default or user-set value to check for value, type, lower, and upper.
#         """
#         meta = self._dict[name]
#         values = meta['values']
#         types = meta['types']
#         lower = meta['lower']
#         upper = meta['upper']
#         dv_flag = meta['dv_flag']

#         if not (value is None and meta['allow_none']):
#             # If only values is declared
#             if values is not None:
#                 if value not in values:
#                     if isinstance(value, string_types):
#                         value = "'{}'".format(value)
#                     raise ValueError("Value ({}) of option '{}' "
#                                      "is not one of {}.".format(value, name, values))
#             # If only types is declared
#             elif types is not None:
#                 if not isinstance(value,  types):
#                     vtype = type(value)
#                     if isinstance(value, string_types):
#                         value = "'{}'".format(value)
#                     raise TypeError("Value ({}) of option '{}' has type of ({}), but "
#                                     "expected type ({}).".format(value, name, vtype, types))

#             if upper is not None:
#                 if value > upper:
#                     raise ValueError("Value ({}) of option '{}' "
#                                      "exceeds maximum allowed value of {}.".format(value, name,
#                                                                                    upper))
#             if lower is not None:
#                 if value < lower:
#                     raise ValueError("Value ({}) of option '{}' "
#                                      "is less than minimum allowed value of {}.".format(value, name,
#                                                                                         lower))
            
#         # General function test
#         # if meta['check_valid'] is not None:
#         #     meta['check_valid'](name, value)

#     def declare(self, name, default=_undefined, values=None, types=None, desc='',
#                 upper=None, lower=None, check_valid=None, allow_none=False, units=None,
#                 dv_flag=False, lb=None,ub=None,scaler=None):
#         r"""
#         Declare an option.

#         The value of the option must satisfy the following:
#         1. If values only was given when declaring, value must be in values.
#         2. If types only was given when declaring, value must satisfy isinstance(value, types).
#         3. It is an error if both values and types are given.

#         Parameters
#         ----------
#         name : str
#             Name of the option.
#         default : object or Null
#             Optional default value that must be valid under the above 3 conditions.
#         values : set or list or tuple or None
#             Optional list of acceptable option values.
#         types : type or tuple of types or None
#             Optional type or list of acceptable option types.
#         desc : str
#             Optional description of the option.
#         upper : float or None
#             Maximum allowable value.
#         lower : float or None
#             Minimum allowable value.
#         check_valid : function or None
#             General check function that raises an exception if value is not valid.
#         allow_none : bool
#             If True, allow None as a value regardless of values or types.
#         """
#         if values is not None and not isinstance(values, (set, list, tuple)):
#             raise TypeError("In declaration of option '%s', the 'values' arg must be of type None,"
#                             " list, or tuple - not %s." % (name, values))
#         if types is not None and not isinstance(types, (type, set, list, tuple)):
#             raise TypeError("In declaration of option '%s', the 'types' arg must be None, a type "
#                             "or a tuple - not %s." % (name, types))

#         if types is not None and values is not None:
#             raise RuntimeError("'types' and 'values' were both specified for option '%s'." %
#                                name)

#         default_provided = default is not _undefined


#         self._dict[name] = {
#             'value': default,
#             'values': values,
#             'types': types,
#             # 'desc': desc,
#             # 'check_valid': check_valid,
#             'has_been_set': default_provided,
#             'allow_none': allow_none,
#             'units' : units,
#             'dv_flag' : dv_flag,
#             'upper': upper,
#             'lower': lower,
#             # 'dv_lower_bound' : lb,
#             # 'dv_upper_bound' : ub,
#             'dv_scaler' : scaler,
#         }

#         # If a default is given, check for validity
#         if default_provided:
#             self._assert_valid(name, default)

#     def undeclare(self, name):
#         """
#         Remove entry from the OptionsDictionary, for classes that don't use that option.

#         Parameters
#         ----------
#         name : str
#             The name of a key, the entry of which will be removed from the internal dictionary.

#         """
#         if name in self._dict:
#             del self._dict[name]

#     def update(self, in_dict):
#         """
#         Update the internal dictionary with the given one.

#         Parameters
#         ----------
#         in_dict : dict
#             The incoming dictionary to add to the internal one.
#         """
#         for name in in_dict:
#             self[name] = in_dict[name]

#     def __iter__(self):
#         """
#         Provide an iterator.

#         Returns
#         -------
#         iterable
#             iterator over the keys in the dictionary.
#         """
#         return iter(self._dict)

#     def __contains__(self, key):
#         """
#         Check if the key is in the local dictionary.

#         Parameters
#         ----------
#         key : str
#             name of the option.

#         Returns
#         -------
#         boolean
#             whether key is in the local dict.
#         """
#         return key in self._dict

#     def __setitem__(self, name, value, units, dv_flag, lower, upper, scaler):
#         """
#         Set an option in the local dictionary.

#         Parameters
#         ----------
#         name : str
#             name of the option.
#         value : -
#             value of the option to be value- and type-checked if declared.
#         """
#         try:
#             meta = self._dict[name]
#         except KeyError:
#             # The key must have been declared.
#             # msg = "Option '{}' cannot be set because it has not been declared."
#             msg = "Unknown variable '{}'. Acceptable variables are " + str(list(self._dict.keys()))
#             raise KeyError(msg.format(name))

#         # if dv_flag is not of type bool
#         if not isinstance(dv_flag,bool):
#             raise TypeError(" 'dv_flag' has type ({}) but must be of type ({}).".format(type(dv_flag),type(False)))

#         if self._read_only:
#             raise KeyError("Tried to set read-only option '{}'.".format(name))

#         self._assert_valid(name, value)

#         meta['value'] = value
#         meta['units'] = units
#         meta['dv_flag'] = dv_flag
#         meta['lower'] = lower
#         meta['upper'] = upper
#         meta['dv_scaler'] = scaler
#         meta['has_been_set'] = True

#     def __getitem__(self, name):
#         """
#         Get an option from the dict or declared default.

#         Parameters
#         ----------
#         name : str
#             name of the option.

#         Returns
#         -------
#         value : -
#             value of the option.
#         """
#         # If the option has been set in this system, return the set value
#         try:
#             meta = self._dict[name]
#             if meta['has_been_set']:
#                 return meta['value']
#             else:
#                 raise RuntimeError("Option '{}' is required but has not been set.".format(name))
#         except KeyError:
#             raise KeyError("Option '{}' cannot be found".format(name))
