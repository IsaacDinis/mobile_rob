import os
import sys
import re

# TODO adjust
# required packages
# numpy

# for testing:
# pip-tools
# rstcheck
# pytest

# pip-tools package:
# its important to pin requirements to get reproducible errors!
# compile a new requirements file (with the latest versions)
# source activate pathEnv
# pip-compile --upgrade
# same as?!:
# pip-compile --output-file requirements.txt requirements.in
# only update the flask package:
# pip-compile --upgrade-package flask
# compile a new requirements file (with versions currently used in the virtual env )
# pip-compile --generate-hashes requirements.in

# do NOT sync. will install ONLY the packages specified! (no more tox etc. installed!)
# pip-sync

# commands
# tox -r to rebuild your tox virtualenvs when you've made changes to requirements setup
# rstcheck *.rst
# tox -r -e py36-codestyle
# tox -r -e py36


def get_version(package):
    """
    Return package version as listed in `__version__` in `__init__.py`.
    """
    init_py = open(os.path.join(package, '__init__.py')).read()
    return re.search("__version__ = ['\"]([^'\"]+)['\"]", init_py).group(1)


def set_version(new_version_number=None, old_version_number=''):
    """
    Set package version as listed in `__version__` in `__init__.py`.
    """
    if new_version_number is None:
        return ValueError

    import fileinput
    import sys

    file = os.path.join('extremitypathfinder', '__init__.py')

    for line in fileinput.input(file, inplace=1):
        if old_version_number in line:
            line = line.replace(old_version_number, new_version_number)
        sys.stdout.write(line)


def convert_version(new_version_input='', old_version='1.0.0'):
    new_version_input = re.search('\d\.\d\.\d+', new_version_input)

    if new_version_input is None:
        return None
    else:
        new_version_input = new_version_input.group()

    # print(new_version_input)

    split_new_version = [int(x) for x in new_version_input.split('.')]
    # print(split_new_version)
    split_old_version = [int(x) for x in old_version.split('.')]
    # print(split_old_version)

    for i in range(3):
        if split_new_version[i] > split_old_version[i]:
            break
        if split_new_version[i] < split_old_version[i]:
            return None

    return new_version_input


def routine(command=None, message='', option1='next', option2='exit'):
    while 1:
        print(message)

        if command:
            print('running command:', command)
            os.system(command)

        print('__________\nDone. Options:')
        print('1)', option1)
        print('2)', option2)
        print('anything else to repeat this step.')
        try:
            inp = int(input())

            if inp == 1:
                print('==============')
                break
            if inp == 2:
                sys.exit()

        except ValueError:
            pass
        print('================')


if __name__ == "__main__":

    print('Do you want to switch to the "dev" branch? Commit before switching branch!')
    print('1) yes, change now.')
    print('2) no, exit')
    print('anything else skip.')
    try:
        inp = int(input())
        if inp == 1:
            os.system('git checkout dev')
            print('==============')
        if inp == 2:
            sys.exit()
    except ValueError:
        pass

    old_version = get_version('extremitypathfinder')

    print('The actual version number is:', old_version)
    print('Enter new version number:')
    version_input = None
    while 1:
        try:
            version_input = input()
        except ValueError:
            pass

        version_number = convert_version(version_input, old_version)
        if version_number is not None:
            set_version(version_number, old_version, )
            break

        print('Invalid version input. Should be of format "x.x.xxx" and higher than the old version.')

    version = get_version('extremitypathfinder')
    print('version number has been set to:', version)
    print('=====================')

    routine(None, 'Remember to write a changelog now for version %s' % version, 'Done. Continue', 'Exit')
    routine(None, 'Are all dependencies written in setup.py, requirements.in/.txt and the Readme?', 'OK. Continue', 'Exit')
    routine(None,
            'Maybe update test routine (requirements.txt) with pip-compile! Commands are written in the beginning of this script',
            'Done. Run tests', 'Exit')

    # print('Enter virtual env name:')
    # virtual env has to be given!
    # virt_env_name = input()
    virt_env_name = 'pathEnv'
    virt_env_act_command = 'source activate ' + virt_env_name.strip() + '; '

    print('___________')
    print('Running TESTS:')

    # routine(virt_env_act_command + "pip-compile requirements.in;pip-sync",
    #         'pinning the requirements.txt and bringing virtualEnv to exactly the specified state:', 'next: build check')

    routine(virt_env_act_command + "rstcheck *.rst", 'checking syntax of all .rst files:', 'next: build check')

    # IMPORTANT: -r flag to rebuild tox virtual env
    # only when dependencies have changed!
    rebuild_flag = ''
    print('when the dependencies (in requirements.txt) have changed enter 1 (-> rebuild tox)')
    try:
        inp = int(input())
        if inp == 1:
            rebuild_flag = ' -r'
    except ValueError:
        pass

    routine(virt_env_act_command + "tox" + rebuild_flag + " -e py37-codestyle",
            'checking syntax, codestyle and imports',
            'continue')

    routine(virt_env_act_command + "tox" + rebuild_flag + " -e py37", 'checking if package is building with tox',
            'continue')

    print('Tests finished.')

    routine(None,
            'Please commit your changes, push and wait if Travis tests build successfully. Only then merge them into the master.',
            'Build successful. Publish and upload now.', 'Exit.')

    # TODO do this automatically, problem are the commit messages (often the same as changelog)
    # git commit --message
    # git push dev

    # if not in master

    # TODO ask to push in master
    # git merge ...

    # TODO switching to master

    # TODO wait for Travis to finish

    print('=================')
    print('PUBLISHING:')

    '''
    ~/.pypirc file required:
    [distutils]
    index-servers =
    pypi
    pypitest
    
    [pypi]
    repository=https://pypi.python.org/pypi
    username=MrMinimal64
    password=****
    
    [pypitest]
    repository=https://testpypi.python.org/pypi
    username=MrMinimal64
    password=your_password
    '''
    routine("python3 setup.py sdist bdist_wheel upload", 'Uploading the package now.')

    # tag erstellen
    routine(None, 'Do you want to create a git release tag?', 'Yes', 'No')

    routine("git tag -a v%s -m 'Version %s'" % (version, version), 'Creating tag', 'Continue')

    routine(None, 'Do you want to push the git release tag?', 'Yes', 'No')
    # in den master pushen
    os.system("git push --tags")

    print('______________')
    print('Publishing Done.')
    print('when the upload didnt work run:')
    print('python3 setup.py bdist_wheel upload')
    print('sudo -H pip3 install extremitypathfinder --upgrade')
