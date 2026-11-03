from setuptools import find_packages, setup

setup(name='limo_evidence_protocol', version='0.1.0', packages=find_packages(),
      data_files=[('share/ament_index/resource_index/packages', ['resource/limo_evidence_protocol']),
                  ('share/limo_evidence_protocol', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='LIMO project', maintainer_email='maintainer@example.invalid',
      description='Shared bounded source-frame retention and accepted-window protocol.',
      license='Apache-2.0')
