
### Modifying the component using custom ESPHome development
If you want to use this component as the basis for further custom development, follow the instructions for [setting up a ESPHome environment](https://developers.esphome.io/contributing/development-environment/).  Create a directory called `config` in the root of the repo, then create a directory `external_components` in the `config` directory.  Copy the `components\mlx90393` folder from this repo to a `mlx90393` directory in the `external_components` directory.  Now create a test yaml file similar to the one above, replacing the external_components section with:
```yaml
external_components:
  - source:
      type: local
      path: external_components
```
To test the code with the esphome runtime use a command line like (making sure the python virtual environment is set, see the instructions):

```
esphome run ./config/test_mlx.yaml --device <serial port name>
```
