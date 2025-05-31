"""
Andrea Favero, 20250603 

'Door at the door' project

Code for config.json management; main elements:
- A subpart of the config.json settings gets exposed to the user via a UI menu.
- In case of changes by the user, those changes need to be merged into the full config.json file.
- Order and nesting of the config.json file are preserved.
- Before saving the new config a backup copy is saved.
- The updated config is saved only if keys/values quantity equals the original.

"""

import ujson as json
import uos, sys

class ConfigManager:
    
    INDENT_UNIT = '    '  # 4-space indentation
    
    def __init__(self, path="config.json"):
         
        self.path = path
        self.backup_path = path + ".bak"
        self.config = {}
        self.order_tree = []
        self.original_data = {}
        self._len_config_keys = 0

    # -------------------- Public API --------------------
    
    def load(self):
        """Load config with preserved order hierarchy
        Returns: config dictionary
        (order_tree is stored internally)
        """
        try:
            # check if file exists
            if self.path not in uos.listdir():
                print(f"CRITICAL: Config file {self.path} not found")
                sys.exit(1)  # Hard exit for boot-critical failure
                
            # proceed with normal loading
            self._config, self._order_tree = self._load_with_order()
            
            # count the number of keys and values
            self._len_config_keys = len(self.flatten_config(self._config))
            
            return self._config
            
        except Exception as e:
            print(f"CRITICAL: Config load failed - {str(e)}")
            sys.exit(1)

    
    
    def save(self, config=None, order_tree=None):
        """Save config with original formatting"""
        config = config if config else self._config
        order = order_tree if order_tree else self._order_tree
        self._safe_write(config, order)
    
    
    
    def flatten_config(self, config=None, prefix="", filter_keys=None):
        """
        Flatten nested config dict into menu-friendly format
        Args:
            config: Dict to flatten (uses self._config if None)
            prefix: Internal use for recursion
            filter_keys: List of keys to include (None=all)
        Returns:
            Dict of flattened key-value pairs
        """
        config = config if config else self._config
        flat = {}
        
        for key, value in config.items():
            full_key = f"{prefix}{key}"
            if isinstance(value, dict):
                flat.update(self.flatten_config(value, f"{full_key}.", filter_keys))
            elif filter_keys is None or full_key in filter_keys:
                flat[full_key] = value
        
        return flat


    # -------------------- Core Functions --------------------
    
    def _load_with_order(self):
        """Your existing load_config_with_key_order as method"""
        def parse_order_hierarchy(lines):
            order_tree = []
            stack = []
            
            for line in lines:
                stripped = line.strip()
                indent = len(line) - len(line.lstrip())
                
                if '"' in line and ':' in line:
                    key = line.split(':', 1)[0].strip(' "\'')
                    is_group = '{' in line.split(':', 1)[1]
                    
                    while stack and indent <= stack[-1][1]:
                        stack.pop()
                    
                    current = order_tree if not stack else stack[-1][0]
                    if is_group:
                        subtree = []
                        current.append((key, subtree))
                        stack.append((subtree, indent))
                    else:
                        current.append((key, None))
            
            return order_tree

        try:
            with open(self.path, "r") as f:
                config = json.load(f)
            
            with open(self.path, "r") as f:
                lines = f.readlines()
                order_tree = parse_order_hierarchy(lines)
            
            return config, order_tree
        except Exception as e:
            print("Config load error:", e)
            return {}, []


    def _safe_write(self, config, order_tree):
        """Your safe_write_config as method"""
        try:
            if self.path in uos.listdir():
                uos.rename(self.path, self.backup_path)
            
            # check if the updated menu has same quantity of kes and values than the original
            if len(self.flatten_config(config)) == self._len_config_keys:
                
                # write the updated config.json file
                with open(self.path, "w") as f:
                    f.write(self._pretty_dumps(config, order_tree))
                
                # if no exceptions the backup copy is removed
                if self.backup_path in uos.listdir():
                    uos.remove(self.backup_path)
            
            else:
                raise Exception("Updated config has different keys/values quantity than original")
            
        except Exception as e:
            print("Config save error:", e)
            
            # if exception, the backup copy is named back as original
            if self.backup_path in uos.listdir():
                uos.rename(self.backup_path, self.path)

    # -------------------- Formatting Utilities --------------------
    
    def _pretty_dumps(self, obj, order_tree=None, level=0):
        """Your pretty_dumps as method with 4-space indent"""
        base_indent = self.INDENT_UNIT * level
        
        if isinstance(obj, dict):
            current_subtree = order_tree if (order_tree and level == 0) else (
                order_tree if isinstance(order_tree, list) else None
            )
            
            items = []
            if current_subtree:
                for key, subtree in current_subtree:
                    if key in obj:
                        items.append((key, obj[key], subtree))
            else:
                items = [(k, v, None) for k, v in obj.items()]
            
            entries = []
            for i, (key, value, subtree) in enumerate(items):
                entry = f'{base_indent}{self.INDENT_UNIT}"{key}": {self._pretty_dumps(value, subtree, level + 1)}'
                entries.append(entry)
                if level == 0 and i < len(items) - 1:
                    entries.append('')
            
            if level == 0:
                content = []
                for entry in entries:
                    if entry == '':
                        content.append('')
                    else:
                        if content and content[-1] != '':
                            content[-1] += ','
                        content.append(entry)
                return '{\n' + '\n'.join(content) + '\n}'
            else:
                return '{\n' + ',\n'.join(entries) + '\n' + base_indent + '}'
        
        elif isinstance(obj, list):
            entries = [f'{base_indent}{self.INDENT_UNIT}{self._pretty_dumps(v, None, level + 1)}' for v in obj]
            return '[\n' + ',\n'.join(entries) + '\n' + base_indent + ']'
        
        else:
            return json.dumps(obj)

    
    # -------------------- Data Transformation Utilities --------------------
    
    def _merge_changes(self, changes):
        """Deep merge changes into current config"""
        
        def merge(dest, src):
            for k, v in src.items():
                if isinstance(v, dict) and isinstance(dest.get(k), dict):
                    merge(dest[k], v)
                else:
                    dest[k] = v
        merge(self._config, changes)

    
    @staticmethod
    def _unflatten(flat_data):
        """Convert flat {'a.b':1} to nested {'a':{'b':1}}"""
        result = {}
        for flat_key, value in flat_data.items():
            keys = flat_key.split('.')
            d = result
            for key in keys[:-1]:
                d = d.setdefault(key, {})
            d[keys[-1]] = value
        return result

