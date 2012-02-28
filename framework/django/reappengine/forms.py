#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#       forms.py
#       
#       Copyright 2012 dominique hunziker <dominique.hunziker@gmail.com>
#       
#       This program is free software; you can redistribute it and/or modify
#       it under the terms of the GNU General Public License as published by
#       the Free Software Foundation; either version 2 of the License, or
#       (at your option) any later version.
#       
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#       GNU General Public License for more details.
#       
#       You should have received a copy of the GNU General Public License
#       along with this program; if not, write to the Free Software
#       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#       MA 02110-1301, USA.
#       
#       

from django import forms
from django.forms import fields
from django.forms.models import inlineformset_factory

from reappengine.models import Package, Node, Param, Interface

class DynamicChoiceField(fields.ChoiceField):
    """ Choice field for which the choices can be dynamically altered """
    def validate(self, value):
        # ingore valiatate() for a ChoiceField and and only check its super
        # class validate() method which is from the class fields.Field;
        # this means that the value does not have to be part of choices and,
        # therefore, the choices can be dynamically altered.
        fields.Field.validate(self, value)

class PackageForm(forms.ModelForm):
    """ Matching form for Package """
    name = DynamicChoiceField()
    
    class Meta:
        model = Package
        exclude = ('user',)
    
    def register(self, pkgs):
        self.fields['name'] = DynamicChoiceField(label='Name', choices=pkgs, required=True, widget=forms.Select())

class NodeForm(forms.ModelForm):
    """ Matching form for Node """
    name = DynamicChoiceField(label='Executable', required=True, widget=forms.Select(attrs={ 'disabled' : 'true' }))
    
    class Meta:
        model = Node
        exclude = ('pkg',)

ParamFormSet = inlineformset_factory(Node, Param, extra=1)

def interfaceFormfieldCallback(field):
    if field.name == 'msgDef':
        return DynamicChoiceField(label='Definition', required=True, widget=forms.Select(attrs={ 'disabled' : 'true' }))
    return field.formfield()

InterfaceFormSet = inlineformset_factory(Package, Interface, extra=1, formfield_callback=interfaceFormfieldCallback)
